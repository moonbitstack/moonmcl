// The linear Kalman filter for the system
//   x_{k+1} = F x_k + w_k      w ~ (0, Q)
//   y_k     = H x_k + v_k      v ~ (0, R)
// It keeps the state estimate x and the error covariance P, alternating
// `predict` and `update`.

///|
/// The linear Kalman filter. Both `predict` and `update` mutate the instance:
/// the estimate and its covariance are the whole of its state.
#derive(@debug.Debug)
pub struct Kf {
  // State estimate.
  mut x : @linear.Vector
  // Error covariance.
  mut p : @linear.Matrix
  // State transition.
  f : @linear.Matrix
  // Process noise covariance.
  q : @linear.Matrix
}

///|
pub extend Kf with @debug.Debug::{to_repr}

///|
/// Build a filter from the initial estimate `x0`, its covariance `p0`, the
/// state transition `f`, and the process noise covariance `q`.
pub fn Kf::kf(
  x0 : @linear.Vector,
  p0 : @linear.Matrix,
  f : @linear.Matrix,
  q : @linear.Matrix,
) -> Kf {
  { x: x0, p: p0, f, q, }
}

///|
/// The current state estimate.
pub fn Kf::state(self : Kf) -> @linear.Vector {
  self.x
}

///|
/// The current error covariance.
pub fn Kf::covariance(self : Kf) -> @linear.Matrix {
  self.p
}

///|
/// Propagate the state and covariance one step: x = F x, P = F P F^T + Q.
pub fn Kf::predict(self : Kf) -> Unit {
  self.x = self.f.mat_vec(self.x)
  let fp = self.f.mat_mat(self.p)
  self.p = fp.mat_mat(self.f.transpose()).add(self.q)
}

///|
/// Fuse a measurement y with measurement matrix H and noise R:
///   S = H P H^T + R,  K = P H^T S^-1,  x += K (y - H x),  P -= K S K^T.
pub fn Kf::update(
  self : Kf,
  y : @linear.Vector,
  h : @linear.Matrix,
  r : @linear.Matrix,
) -> Unit {
  let hp = h.mat_mat(self.p)
  let s = hp.mat_mat(h.transpose()).add(r)
  let k = hp.transpose().mat_mat(s.inv())
  let innovation = y.sub(h.mat_vec(self.x))
  self.x = self.x.add(k.mat_vec(innovation))
  let kh = k.mat_mat(h)
  self.p = self.p.sub(kh.mat_mat(hp))
}
