// A second opinion on the linear filter and on the backward pass. filterpy is the
// primary reference for both; pykalman runs the same two recursions with its own
// code, so these cases are an independent check rather than a repeat.
//
// The reference updates before it predicts, so the first step here corrects
// without a predict; after that the two run the same predict-then-correct order.
// The smoothed figures are compared against the reference's own smoothed output
// for the same record, which is what the primary table does too.

///|
/// Build the discrete plant a case runs on.
fn kf_case_model(
  case : KfPykalmanCase,
) -> @contract.StateSpace raise @linear.Refused {
  @contract.StateSpace::discrete(
    a=mat_of(case.transition, case.n, case.n),
    b=@linear.Mat::zeros(case.n, 1),
    c=mat_of(case.observation, case.m, case.n),
    d=@linear.Mat::zeros(case.m, 1),
    dt=1.0,
  )
}

///|
test "the linear filter follows a second implementation" {
  let cases = kf_pykalman_cases()
  assert_eq(cases.length() >= 3, true)
  for case in cases {
    let filter = @estimation.Kalman::new(
      model=kf_case_model(case),
      process=mat_of(case.process, case.n, case.n),
      measurement=mat_of(case.noise, case.m, case.m),
      state=vec_of(case.prior),
      covariance=mat_of(case.covariance, case.n, case.n),
    ) catch {
      _ => abort("the reference accepted this plant")
    }
    let mut first = true
    let mut k = 0
    for measurement in case.measurements {
      if !first {
        filter.predict(vec_of([0.0]), 1.0)
      }
      first = false
      filter.correct(vec_of(measurement)) catch {
        _ => abort("the reference accepted this measurement")
      }
      assert_eq(same_vec(filter.state(), case.filtered[k], 1.0e-9), true)
      assert_eq(
        same_mat(filter.covariance(), case.filtered_covariance[k], 1.0e-9),
        true,
      )
      k = k + 1
    }
  }
}

///|
test "the backward pass follows a second implementation" {
  let cases = kf_pykalman_cases()
  for case in cases {
    let steps = case.filtered.length()
    let states = case.filtered.map(fn(row) { vec_of(row) })
    let covariances = case.filtered_covariance.map(fn(row) {
      mat_of(row, case.n, case.n)
    })
    let transitions = []
    for _ in 1..<steps {
      transitions.push(mat_of(case.transition, case.n, case.n))
    }
    let (smoothed_states, smoothed_covariances) = @estimation.rts(
      states~,
      covariances~,
      transitions~,
      process=mat_of(case.process, case.n, case.n),
    )
    for k in 0..<steps {
      assert_eq(same_vec(smoothed_states[k], case.smoothed[k], 1.0e-9), true)
      assert_eq(
        same_mat(smoothed_covariances[k], case.smoothed_covariance[k], 1.0e-9),
        true,
      )
    }
  }
}
