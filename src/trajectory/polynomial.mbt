///|
/// A polynomial in time, `position = sum(coefficients[i] * t^i)`.
///
/// A polynomial is the profile to reach for when the boundary conditions are the
/// whole requirement: it matches a position, a velocity and an acceleration at
/// both ends exactly, and it does not care about a jerk limit on the way.
#derive(Debug)
pub(all) struct Polynomial {
  coefficients : Array[Double]
  duration : Double
}

///|
/// The value and its derivatives at `t`, by Horner's rule.
fn Polynomial::evaluate(
  self : Polynomial,
  t : Double,
) -> (Double, Double, Double, Double) {
  let n = self.coefficients.length()
  let mut position = 0.0
  let mut velocity = 0.0
  let mut acceleration = 0.0
  let mut jerk = 0.0
  let mut index = n - 1
  while index >= 0 {
    let coefficient = self.coefficients[index]
    jerk = jerk * t + acceleration * 3.0
    acceleration = acceleration * t + velocity * 2.0
    velocity = velocity * t + position
    position = position * t + coefficient
    index = index - 1
  }
  (position, velocity, acceleration, jerk)
}

///|
/// A cubic from one boundary state to another over `duration`.
///
/// A cubic matches the two positions and the two velocities exactly; its
/// acceleration jumps at both ends, which is what a drive without a jerk limit
/// does anyway.
pub fn cubic(
  start~ : Double,
  end~ : Double,
  start_velocity~ : Double,
  end_velocity~ : Double,
  duration~ : Double,
) -> Polynomial raise @linear.Refused {
  if duration <= 0.0 {
    raise @linear.Refused::Range(got=duration, low=1.0e-12, high=1.0e12)
  }
  let t = duration
  // Solve for the two higher coefficients; the first two come from the boundary
  // position and velocity.
  let matrix = @linear.Mat::of([[t * t, t * t * t], [2.0 * t, 3.0 * t * t]])
  let right = @linear.Mat::of([
    [end - start - start_velocity * t],
    [end_velocity - start_velocity],
  ])
  let solved = @linear.solve(matrix, right)
  {
    coefficients: [start, start_velocity, solved.get(0, 0), solved.get(1, 0)],
    duration,
  }
}

///|
/// A quintic from one boundary state to another over `duration`.
///
/// A quintic also matches the two accelerations, so the acceleration is
/// continuous with whatever profile came before and whatever comes after.
pub fn quintic(
  start~ : Double,
  end~ : Double,
  start_velocity~ : Double,
  start_acceleration~ : Double,
  end_velocity~ : Double,
  end_acceleration~ : Double,
  duration~ : Double,
) -> Polynomial raise @linear.Refused {
  if duration <= 0.0 {
    raise @linear.Refused::Range(got=duration, low=1.0e-12, high=1.0e12)
  }
  let t = duration
  let quadratic = 0.5 * start_acceleration
  let matrix = @linear.Mat::of([
    [t * t * t, t * t * t * t, t * t * t * t * t],
    [3.0 * t * t, 4.0 * t * t * t, 5.0 * t * t * t * t],
    [6.0 * t, 12.0 * t * t, 20.0 * t * t * t],
  ])
  let right = @linear.Mat::of([
    [end - start - start_velocity * t - quadratic * t * t],
    [end_velocity - start_velocity - 2.0 * quadratic * t],
    [end_acceleration - 2.0 * quadratic],
  ])
  let solved = @linear.solve(matrix, right)
  {
    coefficients: [
      start,
      start_velocity,
      quadratic,
      solved.get(0, 0),
      solved.get(1, 0),
      solved.get(2, 0),
    ],
    duration,
  }
}

///|
pub impl @contract.Trajectory for Polynomial with fn duration(self) {
  self.duration
}

///|
pub impl @contract.Trajectory for Polynomial with fn sample(self, t) {
  let (position, velocity, acceleration, jerk) = self.evaluate(t)
  @contract.Sample::new(position~, velocity~, acceleration~, jerk~)
}

///|
pub extend Polynomial with @contract.Trajectory::{duration, sample}

///|
pub extend Polynomial with @debug.Debug::{to_repr}
