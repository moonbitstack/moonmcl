// The linear Kalman filter tracking a constant value through a random walk:
// it alternates predict/update and the estimate converges to the measured
// value while the variance settles.
//
//     moon run src/examples/04-estimation

///|
#coverage.skip
fn main {
  let x0 = @mctl.Vector::of([0.0])
  let p0 = @mctl.Matrix::of([[1.0]])
  let f = @mctl.Matrix::of([[1.0]])
  let q = @mctl.Matrix::of([[0.01]])
  let kf = @mctl.Kf::kf(x0, p0, f, q)
  let h = @mctl.Matrix::of([[1.0]])
  let r = @mctl.Matrix::of([[0.1]])
  let truth = 2.0
  for k in 0..<300 {
    kf.predict()
    // A measurement of the constant value 2.0.
    let y = @mctl.Vector::of([truth])
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
