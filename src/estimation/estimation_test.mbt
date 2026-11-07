// Tests for the estimation module: the linear Kalman filter (variance and
// estimate convergence on a 1-D random walk), and the extended Kalman filter
// (position / velocity tracking of a constant-velocity target).

///|
test "kf variance stays bounded and positive" {
  // 1-D random walk: x' = x + w, y = x + v, Q = 0.01, R = 0.1.
  let x0 = @linear.Vector::of([0.0])
  let p0 = @linear.Matrix::of([[1.0]])
  let f = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[0.01]])
  let kf = @estimation.Kf::kf(x0, p0, f, q)
  let h = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[0.1]])
  let mut min_p = 1.0e30
  for _ in 0..<200 {
    kf.predict()
    let y = @linear.Vector::of([1.0 + 0.01])
    kf.update(y, h, r)
    let p = kf.covariance().at(0, 0).unwrap()
    if p < min_p {
      min_p = p
    }
  }
  let p = kf.covariance().at(0, 0).unwrap()
  assert_eq(p > 0.0, true)
  assert_eq(p < 1.0, true)
}

///|
test "kf estimate converges to the measured constant" {
  let x0 = @linear.Vector::of([0.0])
  let p0 = @linear.Matrix::of([[1.0]])
  let f = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[0.01]])
  let kf = @estimation.Kf::kf(x0, p0, f, q)
  let h = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[0.1]])
  for _ in 0..<300 {
    kf.predict()
    let y = @linear.Vector::of([2.0])
    kf.update(y, h, r)
  }
  let est = kf.state().x()
  assert_eq((est - 2.0).abs() < 0.1, true)
}

///|
test "ekf tracks a constant velocity target" {
  // Position + velocity target moving at 1.0; the EKF predicts with the
  // constant-velocity model and updates with a noisy position measurement.
  let x0 = @linear.Vector::of([0.0, 0.0])
  let p0 = @linear.Matrix::of([[1.0, 0.0], [0.0, 1.0]])
  let q = @linear.Matrix::of([[0.01, 0.0], [0.0, 0.01]])
  let dt = 0.1
  let f = (x : @linear.Vector) => {
    @linear.Vector::of([x.x() + x.y() * dt, x.y()])
  }
  let h = (x : @linear.Vector) => @linear.Vector::of([x.x()])
  let ekf = @estimation.EKF::ekf(x0, p0, q)
  let r = @linear.Matrix::of([[0.05]])
  // Simulate 60 steps; the true position is k * dt with velocity 1.
  let mut pos = 0.0
  let vel = 1.0
  for _ in 0..<60 {
    ekf.predict(f)
    pos = pos + vel * dt
    let y = @linear.Vector::of([pos])
    ekf.update(y, h, r)
  }
  let est = ekf.state()
  assert_eq((est.x() - pos).abs() < 0.2, true)
  assert_eq((est.y() - vel).abs() < 0.2, true)
}
