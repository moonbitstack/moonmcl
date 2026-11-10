// The two single-axis trackers. Both are cheap on purpose: a complementary
// filter costs one multiply, an alpha-beta-gamma tracker three, and either fits
// inside a control period that has no room for a covariance.

///|
/// A complementary filter for one angle: the gyro is trusted for the fast
/// motion, the measurement for the slow drift.
///
/// `alpha` is the fraction of the integrated rate kept each period. At `0.98`
/// with a period of a hundredth of a second the time constant is about half a
/// second, which is the usual balance for a hobby-grade inertial sensor.
#derive(Debug)
pub(all) struct Complementary {
  alpha : Double
  mut angle : Double
}

///|
/// A complementary filter with the given blend and starting angle.
pub fn Complementary::new(
  alpha~ : Double = 0.98,
  angle~ : Double = 0.0,
) -> Complementary raise @linear.Refused {
  if alpha < 0.0 || alpha > 1.0 {
    raise @linear.Refused::Range(got=alpha, low=0.0, high=1.0)
  }
  { alpha, angle, }
}

///|
/// The corrected angle after one period.
pub fn Complementary::update(
  self : Complementary,
  rate~ : Double,
  measurement~ : Double,
  dt~ : Double,
) -> Double {
  self.angle = self.alpha * (self.angle + rate * dt) +
    (1.0 - self.alpha) * measurement
  self.angle
}

///|
/// The current angle.
pub fn Complementary::angle(self : Complementary) -> Double {
  self.angle
}

///|
/// An alpha-beta-gamma tracker: position, velocity and acceleration of one axis
/// from a position measurement.
///
/// The three gains trade noise against lag the way a Kalman filter does with a
/// fixed model: `alpha` corrects the position, `beta` the velocity and `gamma`
/// the acceleration. The preset is the critically damped set for a steady-state
/// model of a constant acceleration.
#derive(Debug)
pub(all) struct AlphaBetaGamma {
  alpha : Double
  beta : Double
  gamma : Double
  mut position : Double
  mut velocity : Double
  mut acceleration : Double
}

///|
/// A tracker with the given gains and starting position.
pub fn AlphaBetaGamma::new(
  alpha~ : Double = 0.5,
  beta~ : Double = 0.2,
  gamma~ : Double = 0.02,
  position~ : Double = 0.0,
) -> AlphaBetaGamma raise @linear.Refused {
  if alpha < 0.0 || alpha > 1.0 {
    raise @linear.Refused::Range(got=alpha, low=0.0, high=1.0)
  }
  if beta < 0.0 || beta > 1.0 {
    raise @linear.Refused::Range(got=beta, low=0.0, high=1.0)
  }
  if gamma < 0.0 || gamma > 1.0 {
    raise @linear.Refused::Range(got=gamma, low=0.0, high=1.0)
  }
  { alpha, beta, gamma, position, velocity: 0.0, acceleration: 0.0, }
}

///|
/// The state after folding in one position measurement.
pub fn AlphaBetaGamma::update(
  self : AlphaBetaGamma,
  measurement~ : Double,
  dt~ : Double,
) -> @contract.Sample {
  // Predict with the acceleration the tracker already believes in, then correct
  // each derivative by its own share of the residual.
  let predicted = self.position +
    self.velocity * dt +
    0.5 * self.acceleration * dt * dt
  let velocity = self.velocity + self.acceleration * dt
  let residual = measurement - predicted
  self.position = predicted + self.alpha * residual
  self.velocity = velocity + self.beta * residual / dt
  self.acceleration = self.acceleration +
    2.0 * self.gamma * residual / (dt * dt)
  @contract.Sample::new(
    position=self.position,
    velocity=self.velocity,
    acceleration=self.acceleration,
  )
}

///|
pub extend Complementary with @debug.Debug::{to_repr}

///|
pub extend AlphaBetaGamma with @debug.Debug::{to_repr}
