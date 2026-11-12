// The default numerical Jacobians of the contracts against an independent
// derivative. The table comes from `scripts/vectors/gen_contract.py`: SciPy's
// complex-step derivative reaches machine precision, so the central difference
// the contracts provide by default has to agree with it to the last digits its
// step size allows.

///|
/// The same nonlinear plant the generator differentiates.
pub struct Plant {}

///|
pub impl @contract.Model for Plant with fn step(self, x, u, dt) {
  ignore(self)
  @linear.Vec::of([
    x.get(0) + dt * (x.get(1) + u.get(0) * @math.sin(x.get(0))),
    x.get(1) +
    dt * (-0.8 * x.get(0) * x.get(1) + u.get(0) * @math.cos(x.get(1))),
  ])
}

///|
pub impl @contract.Model for Plant with fn observe(self, x) {
  ignore(self)
  @linear.Vec::of([
    (x.get(0) * x.get(0) + x.get(1) * x.get(1) + 1.0).sqrt(),
    @math.exp(-0.1 * x.get(0)) * @math.sin(x.get(1)),
  ])
}

///|
pub extend Plant with @contract.Model::{
  step,
  observe,
  step_jacobian,
  observe_jacobian,
}

///|
fn jacobian_close(
  matrix : @linear.Mat,
  expected : Array[Double],
  rows : Int32,
  columns : Int32,
  tolerance~ : Double = 1.0e-8,
) -> Bool {
  for i in 0..<rows {
    for j in 0..<columns {
      if (matrix.get(i, j) - expected[i * columns + j]).abs() > tolerance {
        return false
      }
    }
  }
  true
}

///|
test "the default state and measurement Jacobians match the reference" {
  let cases = jacobian_cases()
  assert_eq(cases.length() >= 9, true)
  let plant = Plant::{ }
  for case in cases {
    let x = @linear.Vec::of(case.x)
    let u = @linear.Vec::of(case.u)
    let state = plant.step_jacobian(x, u, case.dt)
    assert_eq(jacobian_close(state, case.step_jacobian, 2, 2), true)
    let measurement = plant.observe_jacobian(x)
    assert_eq(jacobian_close(measurement, case.observe_jacobian, 2, 2), true)
  }
}
