///|
/// A linear Kalman filter for a discrete state-space plant.
pub(all) struct Kalman {
  model : @contract.StateSpace
  process : @linear.Mat
  measurement : @linear.Mat
  state : @linear.Vec
  covariance : @linear.Mat
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
  abort("moonmcl: Kalman::new is not implemented yet")
}

///|
pub impl @contract.Estimator for Kalman with fn predict(self, u, dt) {
  abort("moonmcl: Kalman::predict is not implemented yet")
}

///|
pub impl @contract.Estimator for Kalman with fn correct(self, z) {
  abort("moonmcl: Kalman::correct is not implemented yet")
}

///|
pub impl @contract.Estimator for Kalman with fn state(self) {
  abort("moonmcl: Kalman::state is not implemented yet")
}

///|
pub impl @contract.Estimator for Kalman with fn covariance(self) {
  abort("moonmcl: Kalman::covariance is not implemented yet")
}

///|
pub extend Kalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
/// An extended Kalman filter: the same recursion as the linear one, with the
/// Jacobians the model supplies.
pub(all) struct ExtendedKalman {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  state : @linear.Vec
  covariance : @linear.Mat
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
  abort("moonmcl: ExtendedKalman::new is not implemented yet")
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn predict(self, u, dt) {
  abort("moonmcl: ExtendedKalman::predict is not implemented yet")
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn correct(self, z) {
  abort("moonmcl: ExtendedKalman::correct is not implemented yet")
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn state(self) {
  abort("moonmcl: ExtendedKalman::state is not implemented yet")
}

///|
pub impl @contract.Estimator for ExtendedKalman with fn covariance(self) {
  abort("moonmcl: ExtendedKalman::covariance is not implemented yet")
}

///|
pub extend ExtendedKalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
/// The spread of the sigma points in the unscented transform.
#derive(Debug)
pub(all) struct UnscentedConfig {
  alpha : Double
  beta : Double
  kappa : Double
}

///|
/// The preset spread: a narrow alpha, the Gaussian beta and the usual kappa.
pub let unscented_config : UnscentedConfig = {
  alpha: 1.0e-3,
  beta: 2.0,
  kappa: 0.0,
}

///|
/// An unscented Kalman filter: the state distribution is carried through the
/// model by a small set of sigma points instead of a linearisation.
pub(all) struct UnscentedKalman {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  state : @linear.Vec
  covariance : @linear.Mat
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
  abort("moonmcl: UnscentedKalman::new is not implemented yet")
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn predict(self, u, dt) {
  abort("moonmcl: UnscentedKalman::predict is not implemented yet")
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn correct(self, z) {
  abort("moonmcl: UnscentedKalman::correct is not implemented yet")
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn state(self) {
  abort("moonmcl: UnscentedKalman::state is not implemented yet")
}

///|
pub impl @contract.Estimator for UnscentedKalman with fn covariance(self) {
  abort("moonmcl: UnscentedKalman::covariance is not implemented yet")
}

///|
pub extend UnscentedKalman with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
/// A particle filter: a cloud of hypotheses propagated through the model and
/// reweighted by the measurement.
pub(all) struct ParticleFilter {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  particles : Array[@linear.Vec]
  weights : Array[Double]
  rng : &@rand.Rng
}

///|
/// A particle filter with `samples` particles drawn around `state`.
pub fn ParticleFilter::new(
  model~ : &@contract.Model,
  process~ : @linear.Mat,
  measurement~ : @linear.Mat,
  state~ : @linear.Vec,
  covariance~ : @linear.Mat,
  rng~ : &@rand.Rng,
  samples~ : Int32 = 1000,
) -> ParticleFilter raise @linear.Refused {
  abort("moonmcl: ParticleFilter::new is not implemented yet")
}

///|
pub impl @contract.Estimator for ParticleFilter with fn predict(self, u, dt) {
  abort("moonmcl: ParticleFilter::predict is not implemented yet")
}

///|
pub impl @contract.Estimator for ParticleFilter with fn correct(self, z) {
  abort("moonmcl: ParticleFilter::correct is not implemented yet")
}

///|
pub impl @contract.Estimator for ParticleFilter with fn state(self) {
  abort("moonmcl: ParticleFilter::state is not implemented yet")
}

///|
pub impl @contract.Estimator for ParticleFilter with fn covariance(self) {
  abort("moonmcl: ParticleFilter::covariance is not implemented yet")
}

///|
pub extend ParticleFilter with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
/// A complementary filter for one angle: the gyro is trusted for the fast
/// motion, the accelerometer for the slow drift.
#derive(Debug)
pub(all) struct Complementary {
  alpha : Double
  angle : Double
}

///|
/// A complementary filter with the given blend and starting angle.
pub fn Complementary::new(
  alpha~ : Double = 0.98,
  angle~ : Double = 0.0,
) -> Complementary raise @linear.Refused {
  abort("moonmcl: Complementary::new is not implemented yet")
}

///|
/// The corrected angle after one period.
pub fn Complementary::update(
  self : Complementary,
  rate~ : Double,
  measurement~ : Double,
  dt~ : Double,
) -> Double {
  abort("moonmcl: Complementary::update is not implemented yet")
}

///|
/// An alpha-beta-gamma tracker: position, velocity and acceleration of one axis
/// from a position measurement.
#derive(Debug)
pub(all) struct AlphaBetaGamma {
  alpha : Double
  beta : Double
  gamma : Double
  position : Double
  velocity : Double
  acceleration : Double
}

///|
/// A tracker with the given gains and starting position.
pub fn AlphaBetaGamma::new(
  alpha~ : Double = 0.5,
  beta~ : Double = 0.2,
  gamma~ : Double = 0.02,
  position~ : Double = 0.0,
) -> AlphaBetaGamma raise @linear.Refused {
  abort("moonmcl: AlphaBetaGamma::new is not implemented yet")
}

///|
/// The state after folding in one position measurement.
pub fn AlphaBetaGamma::update(
  self : AlphaBetaGamma,
  measurement~ : Double,
  dt~ : Double,
) -> @contract.Sample {
  abort("moonmcl: AlphaBetaGamma::update is not implemented yet")
}

///|
/// The Rauch-Tung-Striebel smoother: the forward filter's estimates refined by
/// a backward pass.
pub fn rts(
  states~ : Array[@linear.Vec],
  covariances~ : Array[@linear.Mat],
  transitions~ : Array[@linear.Mat],
  process~ : @linear.Mat,
) -> (Array[@linear.Vec], Array[@linear.Mat]) raise @linear.Refused {
  abort("moonmcl: rts is not implemented yet")
}

///|
pub extend UnscentedConfig with @debug.Debug::{to_repr}

///|
pub extend Complementary with @debug.Debug::{to_repr}

///|
pub extend AlphaBetaGamma with @debug.Debug::{to_repr}
