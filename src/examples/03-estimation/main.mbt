// An extended Kalman filter following a pendulum from a noisy angle measurement,
// with the plant written here rather than in the library.
//
//     moon run src/examples/03-estimation

///|
#coverage.skip
fn main raise {
  let model : &@mctl.Model = Pendulum::{ length: 1.0, damping: 0.1, }
  let filter = @mctl.ExtendedKalman::new(
    model~,
    process=@mctl.Mat::diagonal(@mctl.Vec::of([1.0e-4, 1.0e-4])),
    measurement=@mctl.Mat::of([[1.0e-2]]),
    state=@mctl.Vec::of([0.0, 0.0]),
    covariance=@mctl.Mat::identity(2),
  )
  let mut truth = @mctl.Vec::of([0.6, 0.0])
  for step in 0..<200 {
    truth = model.step(truth, @mctl.Vec::of([0.0]), 0.01)
    // The sensor sees the angle with a little noise of its own.
    let noise = if step % 3 == 0 { 0.01 } else { -0.005 }
    filter.predict(@mctl.Vec::of([0.0]), 0.01)
    filter.correct(@mctl.Vec::of([truth.get(0) + noise]))
    if step % 40 == 0 {
      println(
        "step " +
        step.to_string() +
        "  true " +
        truth.get(0).to_string() +
        "  estimated " +
        filter.state().get(0).to_string(),
      )
    }
  }
}

///|
/// A damped pendulum, integrated one step at a time.
pub struct Pendulum {
  length : Double
  damping : Double
}

///|
pub impl @mctl.Model for Pendulum with fn step(self, x, u, dt) {
  ignore(u)
  let theta = x.get(0)
  let omega = x.get(1)
  let acceleration = -9.81 / self.length * @math.sin(theta) -
    self.damping * omega
  @mctl.Vec::of([theta + omega * dt, omega + acceleration * dt])
}

///|
pub impl @mctl.Model for Pendulum with fn observe(self, x) {
  ignore(self)
  @mctl.Vec::of([x.get(0)])
}
