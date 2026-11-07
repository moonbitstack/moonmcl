// The extended Kalman filter for the nonlinear system
//   x_{k+1} = f(x_k) + w_k
//   y_k     = h(x_k) + v_k
// `predict` and `update` take the nonlinear model for that step, so the same
// filter instance can be reused across changing dynamics; the Jacobians of
// `f` and `h` are estimated by central differences, so callers supply only the
// models. A control input can be baked into `f` with a closure.

///|
/// The extended Kalman filter. The models are passed to `predict` and `update`
/// rather than stored, and their Jacobians are formed by central differences,
/// so the filter itself only holds the estimate, its covariance and the noise.
#derive(@debug.Debug)
pub struct EKF {
  // State estimate.
  mut x : @linear.Vector
  // Error covariance.
  mut p : @linear.Matrix
  // Process noise covariance.
  q : @linear.Matrix
}

///|
pub extend EKF with @debug.Debug::{to_repr}

///|
/// Build a filter from the initial estimate `x0`, its covariance `p0`, and the
/// process noise covariance `q`. The models `f` and `h` are supplied step by
/// step instead, so one instance can follow changing dynamics.
pub fn EKF::ekf(
  x0 : @linear.Vector,
  p0 : @linear.Matrix,
  q : @linear.Matrix,
) -> EKF {
  { x: x0, p: p0, q, }
}

///|
/// The current state estimate.
pub fn EKF::state(self : EKF) -> @linear.Vector {
  self.x
}

///|
/// The current error covariance.
pub fn EKF::covariance(self : EKF) -> @linear.Matrix {
  self.p
}

///|
/// Central-difference Jacobian of `g` at `x`: an `out x in` matrix where
/// `out = g(x).dim()` and `in = x.dim()`.
fn jacobian(
  g : (@linear.Vector) -> @linear.Vector,
  x : @linear.Vector,
) -> @linear.Matrix {
  let n = x.dim()
  let m = g(x).dim()
  let jac = @linear.Matrix::zeros(m, n)
  for j in 0..<n {
    let base = x.get(j).unwrap_or(0.0)
    let step = (1.0e-6 * base.abs()).max(1.0e-6)
    let xp = x.clone()
    xp.set(j, base + step)
    let xm = x.clone()
    xm.set(j, base - step)
    let fp = g(xp)
    let fm = g(xm)
    for i in 0..<m {
      jac.set(
        i,
        j,
        (fp.get(i).unwrap_or(0.0) - fm.get(i).unwrap_or(0.0)) / (2.0 * step),
      )
    }
  }
  jac
}

///|
/// Propagate with model `f`: F = df/dx, x = f(x), P = F P F^T + Q.
pub fn EKF::predict(self : EKF, f : (@linear.Vector) -> @linear.Vector) -> Unit {
  let fj = jacobian(f, self.x)
  self.x = fj.mat_vec(self.x)
  let fp = fj.mat_mat(self.p)
  self.p = fp.mat_mat(fj.transpose()).add(self.q)
}

///|
/// Fuse a measurement y with model `h` and noise R: H = dh/dx,
/// S = H P H^T + R, K = P H^T S^-1, and the Joseph-form covariance update
/// P = (I - K H) P (I - K H)^T + K R K^T, which stays symmetric positive
/// definite.
pub fn EKF::update(
  self : EKF,
  y : @linear.Vector,
  h : (@linear.Vector) -> @linear.Vector,
  r : @linear.Matrix,
) -> Unit {
  let hj = jacobian(h, self.x)
  let hp = hj.mat_mat(self.p)
  let s = hp.mat_mat(hj.transpose()).add(r)
  let k = hp.transpose().mat_mat(s.inv())
  let innovation = y.sub(hj.mat_vec(self.x))
  self.x = self.x.add(k.mat_vec(innovation))
  let i = @linear.Matrix::identity(self.p.rows)
  let ikh = i.sub(k.mat_mat(hj))
  self.p = ikh
    .mat_mat(self.p)
    .mat_mat(ikh.transpose())
    .add(k.mat_mat(r).mat_mat(k.transpose()))
}
