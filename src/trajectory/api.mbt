///|
/// A trapezoidal velocity profile: accelerate, cruise, decelerate.
///
/// It is the cheapest profile that respects a velocity and an acceleration
/// limit, and the one that leaves a step in the acceleration at each corner.
pub(all) struct Trapezoid {
  distance : Double
  v_max : Double
  a_max : Double
  start_velocity : Double
  end_velocity : Double
  duration : Double
  ramp_time : Double
  cruise_time : Double
  peak_velocity : Double
}

///|
/// A trapezoid over `distance` with the given limits and boundary velocities.
pub fn trapezoid(
  distance~ : Double,
  v_max~ : Double,
  a_max~ : Double,
  start_velocity~ : Double = 0.0,
  end_velocity~ : Double = 0.0,
) -> Trapezoid raise @linear.Refused {
  abort("moonmcl: trapezoid is not implemented yet")
}

///|
pub impl @contract.Trajectory for Trapezoid with fn duration(self) {
  abort("moonmcl: Trapezoid::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for Trapezoid with fn sample(self, t) {
  abort("moonmcl: Trapezoid::sample is not implemented yet")
}

///|
pub extend Trapezoid with @contract.Trajectory::{duration, sample}

///|
/// A jerk-limited S-curve profile: seven phases of constant jerk, so the
/// acceleration is continuous and the jerk is bounded.
pub(all) struct Scurve {
  distance : Double
  v_max : Double
  a_max : Double
  j_max : Double
  start_velocity : Double
  start_acceleration : Double
  end_velocity : Double
  end_acceleration : Double
  phases : Array[Double]
  jerks : Array[Double]
  duration : Double
}

///|
/// An S-curve over `distance` with the given limits and boundary conditions.
pub fn scurve(
  distance~ : Double,
  v_max~ : Double,
  a_max~ : Double,
  j_max~ : Double,
  start_velocity~ : Double = 0.0,
  start_acceleration~ : Double = 0.0,
  end_velocity~ : Double = 0.0,
  end_acceleration~ : Double = 0.0,
) -> Scurve raise @linear.Refused {
  abort("moonmcl: scurve is not implemented yet")
}

///|
pub impl @contract.Trajectory for Scurve with fn duration(self) {
  abort("moonmcl: Scurve::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for Scurve with fn sample(self, t) {
  abort("moonmcl: Scurve::sample is not implemented yet")
}

///|
pub extend Scurve with @contract.Trajectory::{duration, sample}

///|
/// A polynomial in time, `position = sum(coefficients[i] * t^i)`.
pub(all) struct Polynomial {
  coefficients : Array[Double]
  duration : Double
}

///|
/// A cubic from one boundary state to another over `duration`.
pub fn cubic(
  start~ : Double,
  end~ : Double,
  start_velocity~ : Double,
  end_velocity~ : Double,
  duration~ : Double,
) -> Polynomial raise @linear.Refused {
  abort("moonmcl: cubic is not implemented yet")
}

///|
/// A quintic from one boundary state to another over `duration`.
pub fn quintic(
  start~ : Double,
  end~ : Double,
  start_velocity~ : Double,
  start_acceleration~ : Double,
  end_velocity~ : Double,
  end_acceleration~ : Double,
  duration~ : Double,
) -> Polynomial raise @linear.Refused {
  abort("moonmcl: quintic is not implemented yet")
}

///|
pub impl @contract.Trajectory for Polynomial with fn duration(self) {
  abort("moonmcl: Polynomial::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for Polynomial with fn sample(self, t) {
  abort("moonmcl: Polynomial::sample is not implemented yet")
}

///|
pub extend Polynomial with @contract.Trajectory::{duration, sample}

///|
/// A natural cubic spline through time-stamped positions, continuous in
/// velocity and acceleration.
pub(all) struct CubicSpline {
  times : Array[Double]
  positions : Array[Double]
  curvatures : Array[Double]
}

///|
/// A natural cubic spline through `(time, position)` knots, which must be
/// ordered in time.
pub fn cubic_spline(
  knots~ : ArrayView[(Double, Double)],
) -> CubicSpline raise @linear.Refused {
  abort("moonmcl: cubic_spline is not implemented yet")
}

///|
pub impl @contract.Trajectory for CubicSpline with fn duration(self) {
  abort("moonmcl: CubicSpline::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for CubicSpline with fn sample(self, t) {
  abort("moonmcl: CubicSpline::sample is not implemented yet")
}

///|
pub extend CubicSpline with @contract.Trajectory::{duration, sample}

///|
/// A clamped uniform B-spline over a control polygon.
pub(all) struct BSpline {
  degree : Int32
  control : Array[Double]
  knots : Array[Double]
  duration : Double
}

///|
/// A clamped B-spline of `degree` over `control`, running for `duration`
/// seconds.
pub fn b_spline(
  control~ : ArrayView[Double],
  degree~ : Int32 = 3,
  duration~ : Double = 1.0,
) -> BSpline raise @linear.Refused {
  abort("moonmcl: b_spline is not implemented yet")
}

///|
pub impl @contract.Trajectory for BSpline with fn duration(self) {
  abort("moonmcl: BSpline::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for BSpline with fn sample(self, t) {
  abort("moonmcl: BSpline::sample is not implemented yet")
}

///|
pub extend BSpline with @contract.Trajectory::{duration, sample}

///|
/// Several single-axis profiles run together so that they start and finish
/// together, the longer axes slowed to the duration of the longest.
pub(all) struct Synced {
  axes : Array[&@contract.Trajectory]
  duration : Double
}

///|
/// A synchronised set from axes that already have a duration.
pub fn Synced::new(
  axes~ : Array[&@contract.Trajectory],
) -> Synced raise @linear.Refused {
  abort("moonmcl: Synced::new is not implemented yet")
}

///|
/// Every axis sampled at the same instant.
pub fn Synced::sample_all(self : Synced, t : Double) -> Array[@contract.Sample] {
  abort("moonmcl: Synced::sample_all is not implemented yet")
}

///|
pub impl @contract.Trajectory for Synced with fn duration(self) {
  abort("moonmcl: Synced::duration is not implemented yet")
}

///|
pub impl @contract.Trajectory for Synced with fn sample(self, t) {
  abort("moonmcl: Synced::sample is not implemented yet")
}

///|
pub extend Synced with @contract.Trajectory::{duration, sample}
