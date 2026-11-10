///|
/// One instant of a single-axis motion, with every derivative a drive may need.
///
/// The jerk is what separates a smooth profile from a trapezoid: a drive that
/// bounds it never excites the resonance the way a step in acceleration does.
#derive(Debug)
pub(all) struct Sample {
  ///
  /// Position, in metres for a linear axis and radians for a joint.
  position : Double
  ///
  /// First derivative of the position.
  velocity : Double
  ///
  /// Second derivative of the position.
  acceleration : Double
  ///
  /// Third derivative of the position.
  jerk : Double
}

///|
/// A sample from its four derivatives.
pub fn Sample::new(
  position~ : Double,
  velocity~ : Double = 0.0,
  acceleration~ : Double = 0.0,
  jerk~ : Double = 0.0,
) -> Sample {
  { position, velocity, acceleration, jerk, }
}

///|
/// A sample that is not moving.
pub fn Sample::still(position : Double) -> Sample {
  { position, velocity: 0.0, acceleration: 0.0, jerk: 0.0, }
}

///|
/// A single-axis motion profile: a duration, and the state at any time in it.
///
/// Sampling outside `[0, duration]` is defined rather than refused: before the
/// start the motion is held at its first sample and after the end at its last,
/// so a caller can run a clock past the end without a branch.
pub(open) trait Trajectory {
  ///|
  /// The length of the motion in seconds.
  fn duration(Self) -> Double

  ///|
  /// The state at `t` seconds from the start.
  fn sample(Self, t : Double) -> Sample
}

///|
pub extend Sample with @debug.Debug::{to_repr}
