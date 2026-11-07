// Modern control in one run: the LQR gain for a discrete double integrator
// (closed loop converges to the origin) and a receding-horizon MPC that drives
// the same plant to a non-zero reference.
//
//     moon run src/examples/06-modern

///|
#coverage.skip
fn main {
  // Discrete double integrator with dt = 0.1, state (position, velocity).
  let dt = 0.1
  let a = @linear.Matrix::of([[1.0, dt], [0.0, 1.0]])
  let b = @linear.Matrix::of([[dt * dt / 2.0], [dt]])
  let q = @linear.Matrix::of([[1.0, 0.0], [0.0, 1.0]])
  let r = @linear.Matrix::of([[0.01]])

  // LQR drives the origin: simulate from (1.0, 0.0) with u = -K x.
  let k = @control.lqr_gain(a, b, q, r)
  let mut x = @linear.Vector::of([1.0, 0.0])
  for _ in 0..<200 {
    let u = k.mat_vec(x)
    x = a.mat_vec(x).sub(b.mat_vec(u))
  }
  println(
    "lqr  : final state = (" +
    x.x().to_string() +
    ", " +
    x.y().to_string() +
    ")",
  )

  // MPC drives to the reference (2.0, 0.0) over a horizon of 10.
  let mpc = @control.Mpc::mpc(a, b, q, r, 10)
  let xref = @linear.Vector::of([2.0, 0.0])
  let mut xs = @linear.Vector::of([0.0, 0.0])
  for _ in 0..<80 {
    let u = mpc.step(xs, xref)
    xs = a.mat_vec(xs).add(b.mat_vec(u))
  }
  println(
    "mpc  : final state = (" +
    xs.x().to_string() +
    ", " +
    xs.y().to_string() +
    ")",
  )
}
