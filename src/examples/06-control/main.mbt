// A model-predictive controller holding a double integrator to a setpoint under
// an input limit, and the same plant under an LQR for comparison.
//
//     moon run src/examples/06-control

///|
#coverage.skip
fn main raise {
  let dt = 0.1
  let a = @mctl.Mat::of([[1.0, dt], [0.0, 1.0]])
  let b = @mctl.Mat::of([[0.5 * dt * dt], [dt]])
  let model = @mctl.StateSpace::discrete(
    a~,
    b~,
    c=@mctl.Mat::identity(2),
    d=@mctl.Mat::zeros(2, 1),
    dt~,
  )
  let mpc = @mctl.Mpc::new(
    model~,
    state_weight=@mctl.Mat::diagonal(@mctl.Vec::of([10.0, 1.0])),
    input_weight=@mctl.Mat::of([[1.0]]),
    config={
      horizon: 15,
      iterations: 500,
      penalty: 2.0,
      input: { low: -0.4, high: 0.4, },
      rate: { low: -10.0, high: 10.0, },
    },
  )
  let target = @mctl.Vec::of([1.0, 0.0])
  let mut state = @mctl.Vec::of([0.0, 0.0])
  for step in 0..<80 {
    let command = mpc.control(state~, target~)
    state = a.apply(state).add(b.apply(command))
    if step % 16 == 0 {
      println(
        "t=" +
        (step.to_double() * dt).to_string() +
        "  x=" +
        state.get(0).to_string() +
        "  v=" +
        state.get(1).to_string() +
        "  u=" +
        command.get(0).to_string(),
      )
    }
  }
}
