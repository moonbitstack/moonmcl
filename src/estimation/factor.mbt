// One square root of a covariance, used wherever a filter has to displace points
// by a covariance. A covariance that is only semi-definite has no Cholesky
// factor, so the fallback is the diagonal, which always exists.

///|
/// A lower triangular square root of `covariance`, or its diagonal when the
/// matrix is not positive definite.
fn matrix_sqrt(covariance : @linear.Mat) -> @linear.Mat {
  let n = covariance.rows
  let jitter = @linear.Mat::identity(n).scale(1.0e-12)
  @linear.cholesky(covariance.add(jitter)) catch {
    _ => {
      let diagonal = @linear.Vec::zeros(n)
      for i in 0..<n {
        let value = covariance.get(i, i)
        diagonal.set(i, if value > 0.0 { value.sqrt() } else { 0.0 })
      }
      @linear.Mat::diagonal(diagonal)
    }
  }
}
