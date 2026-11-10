// Tests for the estimation package. The linear filter is checked against its own
// recursion on a plant whose answer is known; the filters over `Model` are
// checked with a plant defined here, which is also the proof that a plant
// written outside this library works with them unchanged.

///|
fn refused(f : () -> Unit raise @linear.Refused) -> @linear.Refused? {
  try {
    f()
    None
  } catch {
    e => Some(e)
  }
}

///|
fn close(a : Double, b : Double, tolerance~ : Double = 1.0e-9) -> Bool {
  (a - b).abs() <= tolerance
}

// --- a plant defined outside the library ----------------------------------

///|
/// A pendulum with viscous damping, integrated one step at a time. It exists only
/// to prove that the filters take a plant they have never seen.
pub struct Pendulum {
  length : Double
  damping : Double
}

///|
pub impl @contract.Model for Pendulum with fn step(self, x, u, dt) {
  let theta = x.get(0)
  let omega = x.get(1)
  let acceleration = -9.81 / self.length * @math.sin(theta) -
    self.damping * omega +
    u.get(0)
  @linear.Vec::of([theta + omega * dt, omega + acceleration * dt])
}

///|
pub impl @contract.Model for Pendulum with fn observe(self, x) {
  ignore(self)
  @linear.Vec::of([x.get(0)])
}

///|
pub extend Pendulum with @contract.Model::{
  step,
  observe,
  step_jacobian,
  observe_jacobian,
}

// --- linear Kalman filter -------------------------------------------------

///|
/// A constant-velocity model sampled at `dt` with a position measurement.
fn constant_velocity(dt : Double) -> @contract.StateSpace {
  {
    a: @linear.Mat::of([[1.0, dt], [0.0, 1.0]]),
    b: @linear.Mat::of([[0.5 * dt * dt], [dt]]),
    c: @linear.Mat::of([[1.0, 0.0]]),
    d: @linear.Mat::zeros(1, 1),
    dt: Some(dt),
  }
}

///|
test "kalman tracks a constant velocity" {
  let filter = @estimation.Kalman::new(
    model=constant_velocity(0.1),
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-4, 1.0e-4])),
    measurement=@linear.Mat::of([[1.0e-2]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::identity(2),
  )
  // A body moving at 2 m/s from the origin, measured every period.
  let mut truth = @linear.Vec::of([0.0, 2.0])
  let model = constant_velocity(0.1)
  for _ in 0..<50 {
    truth = model.step(truth, @linear.Vec::of([0.0]), 0.1)
    filter.predict(@linear.Vec::of([0.0]), 0.1)
    filter.correct(@linear.Vec::of([truth.get(0)]))
  }
  assert_eq(close(filter.state().get(0), truth.get(0), tolerance=1.0e-2), true)
  assert_eq(close(filter.state().get(1), 2.0, tolerance=1.0e-2), true)
}

///|
test "kalman keeps its covariance symmetric and shrinking" {
  let filter = @estimation.Kalman::new(
    model=constant_velocity(0.1),
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-4, 1.0e-4])),
    measurement=@linear.Mat::of([[1.0e-2]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::identity(2),
  )
  for k in 0..<20 {
    filter.predict(@linear.Vec::of([0.0]), 0.1)
    filter.correct(@linear.Vec::of([k.to_double() * 0.2]))
  }
  let covariance = filter.covariance()
  assert_eq((covariance.get(0, 1) - covariance.get(1, 0)).abs() < 1.0e-15, true)
  assert_eq(covariance.get(0, 0) < 1.0, true)
}

///|
test "kalman refuses a continuous plant" {
  let continuous = @contract.StateSpace::continuous(
    a=@linear.Mat::of([[0.0, 1.0], [0.0, 0.0]]),
    b=@linear.Mat::of([[0.0], [1.0]]),
    c=@linear.Mat::of([[1.0, 0.0]]),
    d=@linear.Mat::zeros(1, 1),
  )
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @estimation.Kalman::new(
          model=continuous,
          process=@linear.Mat::identity(2),
          measurement=@linear.Mat::of([[1.0]]),
          state=@linear.Vec::of([0.0, 0.0]),
          covariance=@linear.Mat::identity(2),
        ),
      )
    }) {
    Some(@linear.Refused::Unreachable) => ()
    _ => abort("expected Unreachable")
  }
}

// --- extended, unscented and particle filters -----------------------------

///|
/// The filters all see the same pendulum and the same measurement of its angle.
fn pendulum_start() -> @linear.Vec {
  @linear.Vec::of([0.5, 0.0])
}

///|
test "extended kalman follows a pendulum" {
  let model : &@contract.Model = Pendulum::{ length: 1.0, damping: 0.1, }
  let filter = @estimation.ExtendedKalman::new(
    model~,
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-6, 1.0e-6])),
    measurement=@linear.Mat::of([[1.0e-4]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::identity(2),
  )
  let mut truth = pendulum_start()
  for _ in 0..<40 {
    truth = model.step(truth, @linear.Vec::of([0.0]), 0.01)
    filter.predict(@linear.Vec::of([0.0]), 0.01)
    filter.correct(@linear.Vec::of([truth.get(0)]))
  }
  assert_eq(close(filter.state().get(0), truth.get(0), tolerance=1.0e-2), true)
}

///|
test "unscented kalman follows a pendulum" {
  let model : &@contract.Model = Pendulum::{ length: 1.0, damping: 0.1, }
  let filter = @estimation.UnscentedKalman::new(
    model~,
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-6, 1.0e-6])),
    measurement=@linear.Mat::of([[1.0e-4]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::identity(2),
  )
  let mut truth = pendulum_start()
  for _ in 0..<40 {
    truth = model.step(truth, @linear.Vec::of([0.0]), 0.01)
    filter.predict(@linear.Vec::of([0.0]), 0.01)
    filter.correct(@linear.Vec::of([truth.get(0)]))
  }
  assert_eq(close(filter.state().get(0), truth.get(0), tolerance=1.0e-2), true)
}

///|
test "particle filter follows a pendulum with a seeded generator" {
  let model : &@contract.Model = Pendulum::{ length: 1.0, damping: 0.1, }
  let rng = @rand.seeded(b"\x01\x02\x03\x04\x05\x06\x07\x08")
  let filter = @estimation.ParticleFilter::new(
    model~,
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-6, 1.0e-6])),
    measurement=@linear.Mat::of([[1.0e-4]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::diagonal(@linear.Vec::of([0.25, 0.25])),
    rng~,
    samples=500,
  )
  let mut truth = pendulum_start()
  for _ in 0..<40 {
    truth = model.step(truth, @linear.Vec::of([0.0]), 0.01)
    filter.predict(@linear.Vec::of([0.0]), 0.01)
    filter.correct(@linear.Vec::of([truth.get(0)]))
  }
  assert_eq(close(filter.state().get(0), truth.get(0), tolerance=5.0e-2), true)
}

///|
test "particle filter refuses a cloud that is too small" {
  let model : &@contract.Model = Pendulum::{ length: 1.0, damping: 0.1, }
  let rng = @rand.seeded(b"\x01\x02\x03\x04\x05\x06\x07\x08")
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @estimation.ParticleFilter::new(
          model~,
          process=@linear.Mat::identity(2),
          measurement=@linear.Mat::of([[1.0]]),
          state=@linear.Vec::of([0.0, 0.0]),
          covariance=@linear.Mat::identity(2),
          rng~,
          samples=1,
        ),
      )
    }) {
    Some(@linear.Refused::Range(_)) => ()
    _ => abort("expected Range")
  }
}

// --- single-axis trackers -------------------------------------------------

///|
test "complementary filter follows the measurement when the rate is wrong" {
  let filter = @estimation.Complementary::new(alpha=0.9, angle=0.0)
  // The gyro claims nothing is moving; the measurement says 1 radian.
  for _ in 0..<200 {
    ignore(filter.update(rate=0.0, measurement=1.0, dt=0.01))
  }
  assert_eq(close(filter.angle(), 1.0, tolerance=1.0e-3), true)
}

///|
test "complementary filter integrates the rate" {
  let filter = @estimation.Complementary::new(alpha=1.0, angle=0.0)
  for _ in 0..<100 {
    ignore(filter.update(rate=1.0, measurement=0.0, dt=0.01))
  }
  assert_eq(close(filter.angle(), 1.0, tolerance=1.0e-9), true)
}

///|
test "alpha beta gamma recovers a constant acceleration" {
  let tracker = @estimation.AlphaBetaGamma::new(
    alpha=0.5,
    beta=0.2,
    gamma=0.02,
    position=0.0,
  )
  let dt = 0.02
  let mut truth = 0.0
  let mut velocity = 0.0
  let acceleration = 2.0
  let mut sample = @contract.Sample::still(0.0)
  for _ in 0..<500 {
    velocity = velocity + acceleration * dt
    truth = truth + velocity * dt
    sample = tracker.update(measurement=truth, dt~)
  }
  assert_eq(close(sample.position, truth, tolerance=1.0e-2), true)
  assert_eq(close(sample.velocity, velocity, tolerance=5.0e-2), true)
  assert_eq(close(sample.acceleration, acceleration, tolerance=5.0e-1), true)
}

// --- smoothing ------------------------------------------------------------

///|
test "rts smoothing does not grow the covariance" {
  let dt = 0.1
  let filter = @estimation.Kalman::new(
    model=constant_velocity(dt),
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-2, 1.0e-2])),
    measurement=@linear.Mat::of([[1.0e-1]]),
    state=@linear.Vec::of([0.0, 0.0]),
    covariance=@linear.Mat::identity(2),
  )
  let states = Array::make(10, @linear.Vec::zeros(2))
  let covariances = Array::make(10, @linear.Mat::identity(2))
  for k in 0..<10 {
    filter.predict(@linear.Vec::of([0.0]), dt)
    // A noisy measurement of a body moving at 1 m/s.
    let noise = if k % 2 == 0 { 0.05 } else { -0.05 }
    filter.correct(@linear.Vec::of([1.0 * dt * k.to_double() + noise]))
    states[k] = filter.state()
    covariances[k] = filter.covariance()
  }
  let (smoothed_states, smoothed_covariances) = @estimation.rts(
    states~,
    covariances~,
    transitions=Array::make(9, constant_velocity(dt).a),
    process=@linear.Mat::diagonal(@linear.Vec::of([1.0e-2, 1.0e-2])),
  )
  assert_eq(smoothed_states.length(), 10)
  for k in 0..<10 {
    assert_eq(
      smoothed_covariances[k].get(0, 0) <= covariances[k].get(0, 0) + 1.0e-12,
      true,
    )
  }
}
