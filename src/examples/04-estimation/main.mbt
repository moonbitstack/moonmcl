// The linear Kalman filter tracking a constant value through a random walk:
// it alternates predict/update and the estimate converges to the measured
// value while the variance settles.
//
//     moon run src/examples/04-estimation

///|
#coverage.skip
fn main {
  let x0 = @linear.Vector::of([0.0])
  let p0 = @linear.Matrix::of([[1.0]])
  let f = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[0.01]])
  let kf = @estimation.Kf::kf(x0, p0, f, q)
  let h = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[0.1]])
  let truth = 2.0
  for k in 0..<300 {
    kf.predict()
    // A measurement of the constant value 2.0.
    let y = @linear.Vector::of([truth])
    kf.update(y, h, r)
    if k % 100 == 0 {
      let est = kf.state().x()
      let sigma = kf.covariance().at(0, 0).unwrap()
      println(
        "k=" +
        k.to_string() +
        "  est=" +
        est.to_string() +
        "  var=" +
        sigma.to_string(),
      )
    }
  }
  println(
    "final estimate=" +
    kf.state().x().to_string() +
    " (truth " +
    truth.to_string() +
    ")",
  )
}
