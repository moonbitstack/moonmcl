// The filters that take any plant: the extended one linearises the model at the
// current estimate, the unscented one carries a small set of sigma points
// through it. Both implement the same `Estimator` contract, so a caller swaps
// one for the other without touching the loop.

///|
/// An extended Kalman filter: the linear recursion with the Jacobians the model
/// supplies, whether analytically or by the contract's default differences.
pub(all) struct ExtendedKalman {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  mut state : @linear.Vec
  mut covariance : @linear.Mat
}

///|
/// An extended Kalman filter for any plant that implements the model contract.
pub fn ExtendedKalman::new(
  model~ : &@contract.Model,
  process~ : @linear.Mat,
  measurement~ : @linear.Mat,
  state~ : @linear.Vec,
  covariance~ : @linear.Mat,
) -> ExtendedKalman raise @linear.Refused {
  let n = state.len()
  if covariance.rows != n || covariance.cols != n {
    raise @linear.Refused::Dimension(got=covariance.rows, want=n)
  }
  if process.rows != n || process.cols != n {
    raise @linear.Refused::Dimension(got=process.rows, want=n)
  }
  let m = measurement.rows
  if measurement.cols != m {
    raise @linear.Refused::Dimension(got=measurement.cols, want=m)
  }
  { model, process, measurement, state: state.clone(), covariance, }
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn predict(self, u, dt) {
  let transition = self.model.step_jacobian(self.state, u, dt)
  self.state = self.model.step(self.state, u, dt)
  let propagated = transition.mul(self.covariance).mul(transition.transpose())
  self.covariance = propagated.add(self.process)
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn correct(self, z) {
  let observation = self.model.observe_jacobian(self.state)
  let predicted = self.model.observe(self.state)
  let innovation = z.sub(predicted)
  let s = observation
    .mul(self.covariance)
    .mul(observation.transpose())
    .add(self.measurement)
  let gain = @linear.solve(s, observation.mul(self.covariance)).transpose()
  let n = self.state.len()
  self.state = self.state.add(gain.apply(innovation))
  let identity = @linear.Mat::identity(n)
  let factor = identity.sub(gain.mul(observation))
  self.covariance = factor
    .mul(self.covariance)
    .mul(factor.transpose())
    .add(gain.mul(self.measurement).mul(gain.transpose()))
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn state(self) {
  self.state.clone()
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn covariance(self) {
  self.covariance
}

///|
pub extend ExtendedKalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
impl @debug.Debug for ExtendedKalman with fn to_repr(self) {
  @debug.Repr::literal(
    "ExtendedKalman(" + @linear.Vec::to_string(self.state) + ")",
  )
}

///|
pub extend ExtendedKalman with @debug.Debug::{to_repr}

///|
/// The spread of the sigma points in the unscented transform.
///
/// `alpha` sets how far the points sit from the mean, `beta` folds in the
/// knowledge that the distribution is Gaussian, and `kappa` is the usual
/// secondary scaling. The preset is the one the reference implementations use.
#derive(Debug)
pub(all) struct UnscentedConfig {
  alpha : Double
  beta : Double
  kappa : Double
}

///|
/// The preset spread: a narrow alpha, the Gaussian beta and a zero kappa.
pub let unscented_config : UnscentedConfig = {
  alpha: 1.0e-3,
  beta: 2.0,
  kappa: 0.0,
}

///|
/// An unscented Kalman filter.
pub(all) struct UnscentedKalman {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  mut state : @linear.Vec
  mut covariance : @linear.Mat
  config : UnscentedConfig
}

///|
/// An unscented Kalman filter for any plant that implements the model contract.
pub fn UnscentedKalman::new(
  model~ : &@contract.Model,
  process~ : @linear.Mat,
  measurement~ : @linear.Mat,
  state~ : @linear.Vec,
  covariance~ : @linear.Mat,
  config~ : UnscentedConfig = unscented_config,
) -> UnscentedKalman raise @linear.Refused {
  let n = state.len()
  if covariance.rows != n || covariance.cols != n {
    raise @linear.Refused::Dimension(got=covariance.rows, want=n)
  }
  if process.rows != n || process.cols != n {
    raise @linear.Refused::Dimension(got=process.rows, want=n)
  }
  { model, process, measurement, state: state.clone(), covariance, config, }
}

///|
/// A sigma point set with the weights that go with it.
priv struct Sigma {
  points : Array[@linear.Vec]
  means : Array[Double]
  covariances : Array[Double]
}

///|
/// The sigma points of `state` and `covariance`.
fn sigma_points(
  state : @linear.Vec,
  covariance : @linear.Mat,
  config : UnscentedConfig,
) -> Sigma {
  let n = state.len()
  let lambda = config.alpha * config.alpha * (n.to_double() + config.kappa) -
    n.to_double()
  let raw = n.to_double() + lambda
  // A spread that collapses would give every sigma point the same weight; the
  // preset never does, but a caller's alpha can.
  let scale = if raw > 1.0e-6 { raw } else { 1.0e-6 }
  let factor = matrix_sqrt(covariance)
  let spread = scale.sqrt()
  let points = Array::make(2 * n + 1, state)
  let means = Array::make(2 * n + 1, 0.0)
  let covariances = Array::make(2 * n + 1, 0.0)
  points[0] = state.clone()
  means[0] = lambda / scale
  covariances[0] = lambda / scale +
    (1.0 - config.alpha * config.alpha + config.beta)
  for i in 0..<n {
    let column = factor.col(i).scale(spread)
    points[1 + i] = state.add(column)
    points[1 + n + i] = state.sub(column)
    means[1 + i] = 1.0 / (2.0 * scale)
    means[1 + n + i] = 1.0 / (2.0 * scale)
    covariances[1 + i] = 1.0 / (2.0 * scale)
    covariances[1 + n + i] = 1.0 / (2.0 * scale)
  }
  { points, means, covariances, }
}

///|
/// The weighted mean of a point set.
fn weighted_mean(
  points : Array[@linear.Vec],
  weights : Array[Double],
) -> @linear.Vec {
  let mut mean = @linear.Vec::zeros(points[0].len())
  for i in 0..<points.length() {
    mean = mean.axpy(weights[i], points[i])
  }
  mean
}

///|
/// The weighted covariance of a point set about `mean`, plus `noise`.
fn weighted_covariance(
  points : Array[@linear.Vec],
  weights : Array[Double],
  mean : @linear.Vec,
  noise : @linear.Mat,
) -> @linear.Mat {
  let n = mean.len()
  let mut out = noise
  for i in 0..<points.length() {
    let deviation = points[i].sub(mean)
    let outer = @linear.Mat::zeros(n, n)
    for r in 0..<n {
      for c in 0..<n {
        outer.set(r, c, deviation.get(r) * deviation.get(c) * weights[i])
      }
    }
    out = out.add(outer)
  }
  out
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn predict(self, u, dt) {
  let set = sigma_points(self.state, self.covariance, self.config)
  let propagated = Array::make(set.points.length(), self.state)
  for i in 0..<set.points.length() {
    propagated[i] = self.model.step(set.points[i], u, dt)
  }
  let mean = weighted_mean(propagated, set.means)
  self.state = mean
  self.covariance = weighted_covariance(
    propagated,
    set.covariances,
    mean,
    self.process,
  )
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn correct(self, z) {
  let set = sigma_points(self.state, self.covariance, self.config)
  let projected = Array::make(set.points.length(), z)
  for i in 0..<set.points.length() {
    projected[i] = self.model.observe(set.points[i])
  }
  let mean = weighted_mean(projected, set.means)
  let innovation_covariance = weighted_covariance(
    projected,
    set.covariances,
    mean,
    self.measurement,
  )
  let n = self.state.len()
  let m = mean.len()
  let cross = @linear.Mat::zeros(n, m)
  for i in 0..<set.points.length() {
    let dx = set.points[i].sub(self.state)
    let dz = projected[i].sub(mean)
    for r in 0..<n {
      for c in 0..<m {
        cross.add_to(r, c, dx.get(r) * dz.get(c) * set.covariances[i])
      }
    }
  }
  let gain = @linear.solve(innovation_covariance.transpose(), cross.transpose()).transpose()
  self.state = self.state.add(gain.apply(z.sub(mean)))
  self.covariance = self.covariance.sub(
    gain.mul(innovation_covariance).mul(gain.transpose()),
  )
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn state(self) {
  self.state.clone()
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn covariance(self) {
  self.covariance
}

///|
pub extend UnscentedKalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
impl @debug.Debug for UnscentedKalman with fn to_repr(self) {
  @debug.Repr::literal(
    "UnscentedKalman(" + @linear.Vec::to_string(self.state) + ")",
  )
}

///|
pub extend UnscentedKalman with @debug.Debug::{to_repr}

///|
pub extend UnscentedConfig with @debug.Debug::{to_repr}
