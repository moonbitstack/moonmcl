// A second opinion on the unscented filter. The primary table comes from filterpy;
// pykalman implements the same recursion separately, so a table over the same
// plant with a fresh measurement sequence is an independent check rather than a
// repeat of the same arithmetic.
//
// The two libraries differ in two places that no configuration closes, so the
// comparison is a trend rather than the last digits: the sigma points are spread
// at alpha 1 here against the reference's own choice, and the matrix square root
// is a symmetric eigen decomposition here against its Cholesky factor. Both are
// legitimate unscented filters, and a nonlinear observation makes the second
// order term depend on which spread the points took, which is why the agreement
// is 5e-3 relative and not 1e-9. The filterpy table, which spreads at 1e-3 where
// that term vanishes, is the one that pins the digits.

///|
/// pykalman's spread for a two-state filter.
fn pykalman_config() -> @estimation.UnscentedConfig {
  { alpha: 1.0, beta: 0.0, kappa: 1.0, }
}

///|
test "the unscented filter follows a second implementation" {
  let cases = pykalman_cases()
  assert_eq(cases.length() >= 3, true)
  for case in cases {
    let filter = @estimation.UnscentedKalman::new(
      model=Cart::{ },
      process=mat_of([0.01, 0.0, 0.0, 0.02], 2, 2),
      measurement=mat_of([0.1], 1, 1),
      state=vec_of(case.prior),
      covariance=mat_of(case.covariance, 2, 2),
      config=pykalman_config(),
    ) catch {
      _ => abort("the reference accepted this prior")
    }
    // The reference updates first and predicts between updates; the same order
    // here is what makes the two records comparable.
    let states = []
    let mut first = true
    for value in case.measurements {
      if !first {
        filter.predict(vec_of([0.0]), case.dt)
      }
      first = false
      filter.correct(vec_of([value])) catch {
        _ => abort("the reference accepted this measurement")
      }
      states.push(filter.state().to_array())
    }
    assert_eq(states.length(), case.states.length())
    for k in 0..<states.length() {
      assert_eq(same_vec(vec_of(states[k]), case.states[k], 5.0e-3), true)
    }
  }
}
