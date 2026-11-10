///|
/// A linear Kalman filter for a discrete state-space plant.
pub(all) struct Kalman {
  model : @contract.StateSpace
  process : @linear.Mat
  measurement : @linear.Mat
  mut state : @linear.Vec
  mut covariance : @linear.Mat
}

///|
/// A Kalman filter for `model` with the given noise covariances and prior.
pub fn Kalman::new(
  model~ : @contract.StateSpace,
  process~ : @linear.Mat,
  measurement~ : @linear.Mat,
  state~ : @linear.Vec,
  covariance~ : @linear.Mat,
) -> Kalman raise @linear.Refused {
  let n = model.a.rows
  let m = model.c.rows
  if !model.is_discrete() {
    raise @linear.Refused::Unreachable
  }
  if state.len() != n {
    raise @linear.Refused::Dimension(got=state.len(), want=n)
  }
  if covariance.rows != n || covariance.cols != n {
    raise @linear.Refused::Dimension(got=covariance.rows, want=n)
  }
  if process.rows != n || process.cols != n {
    raise @linear.Refused::Dimension(got=process.rows, want=n)
  }
  if measurement.rows != m || measurement.cols != m {
    raise @linear.Refused::Dimension(got=measurement.rows, want=m)
  }
  { model, process, measurement, state: state.clone(), covariance, }
}

///|
pub impl @contract.Estimator for Kalman with fn predict(self, u, dt) {
  ignore(dt)
  let n = self.state.len()
  self.state = self.model.a.apply(self.state).add(self.model.b.apply(u))
  let propagated = self.model.a
    .mul(self.covariance)
    .mul(self.model.a.transpose())
  self.covariance = propagated.add(self.process)
  ignore(n)
}

///|
pub impl @contract.Estimator for Kalman with fn correct(self, z) {
  let c = self.model.c
  let innovation = z.sub(c.apply(self.state))
  let s = c.mul(self.covariance).mul(c.transpose()).add(self.measurement)
  // k = p c' s^-1, computed as (s^-1 (c p))' because s is the square one.
  let gain = @linear.solve(s, c.mul(self.covariance)).transpose()
  let n = self.state.len()
  self.state = self.state.add(gain.apply(innovation))
  // Joseph form: (i - k c) p (i - k c)' + k r k'.
  let identity = @linear.Mat::identity(n)
  let factor = identity.sub(gain.mul(c))
  let left = factor.mul(self.covariance).mul(factor.transpose())
  let right = gain.mul(self.measurement).mul(gain.transpose())
  self.covariance = left.add(right)
}

///|
pub impl @contract.Estimator for Kalman with fn state(self) {
  self.state.clone()
}

///|
pub impl @contract.Estimator for Kalman with fn covariance(self) {
  self.covariance
}

///|
pub extend Kalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}
