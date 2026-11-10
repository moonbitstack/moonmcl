///|
/// A feedback law: the input to apply for a reference and a measurement.
///
/// The combinators in `control` (cascade, saturation, rate limiting) take any
/// `&Controller`, so a controller written outside this library composes the same
/// way the ones here do.
pub(open) trait Controller {
  ///|
  /// The input for one control period of `dt` seconds.
  fn update(
    Self,
    reference : @linear.Vec,
    measurement : @linear.Vec,
    dt : Double,
  ) -> @linear.Vec

  ///|
  /// Drop the accumulated state (an integrator, a stored derivative) and start
  /// again from the next measurement.
  fn reset(Self) -> Unit
}
