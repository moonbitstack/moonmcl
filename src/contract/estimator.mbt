///|
/// A state estimator: propagate on the input, then fold in a measurement.
///
/// The two steps are separate because a sensor does not always deliver on every
/// control period; a caller that has no measurement that cycle calls `predict`
/// alone.
pub(open) trait Estimator {
  ///|
  /// Propagate the estimate and its covariance by `dt` seconds under `u`.
  fn predict(Self, u : @linear.Vec, dt : Double) -> Unit

  ///|
  /// Fold in the measurement `z`.
  fn correct(Self, z : @linear.Vec) -> Unit raise @linear.Refused

  ///|
  /// The current state estimate.
  fn state(Self) -> @linear.Vec

  ///|
  /// The current state covariance.
  fn covariance(Self) -> @linear.Mat
}
