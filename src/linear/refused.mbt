// Every failure this library reports is this one type. The variants are named
// after the thing that is wrong, so a caller can branch on what to fix without
// reading a message.

///|
/// A refusal: the input was not acceptable, or a computation could not reach a
/// result.
///
/// `Dimension` carries a size that does not fit the operation; `Singular` and
/// `NotPositiveDefinite` name the pivot index that failed; `Exceeded` reports an
/// iteration budget that ran out with the value it reached.
pub(all) suberror Refused {
  ///
  /// A vector or matrix has `got` entries where the operation needs `want`.
  Dimension(got~ : Int32, want~ : Int32)
  ///
  /// An operation that needs at least one entry was handed an empty operand.
  Empty
  ///
  /// A pivot vanished at column `at`, so the matrix has no inverse.
  Singular(at~ : Int32)
  ///
  /// A Cholesky pivot at column `at` was not positive, so the matrix is not
  /// symmetric positive definite.
  NotPositiveDefinite(at~ : Int32)
  ///
  /// A gain of `got` is outside the range the algorithm allows.
  Gain(got~ : Double)
  ///
  /// A value of `got` is outside the band `want` gives, expressed as the
  /// inclusive limits `[low, high]`.
  Range(got~ : Double, low~ : Double, high~ : Double)
  ///
  /// An iteration stopped at `limit` steps with `got` steps still to go.
  Exceeded(limit~ : Int32, got~ : Int32)
  ///
  /// No exact solution exists: a pose the mechanism cannot reach, or a
  /// boundary condition the profile cannot meet.
  Unreachable
}
