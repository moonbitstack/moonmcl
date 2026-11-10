///|
/// How a PID stops its integrator while the output is saturated.
#derive(Debug)
pub(all) enum AntiWindup {
  ///
  /// The integrator keeps integrating; a long saturation then has to be unwound.
  Off
  ///
  /// The integrator stops while the unsaturated output would push further into
  /// the limit.
  Conditional
  ///
  /// The integrator is pulled back by the difference between the saturated and
  /// the unsaturated output, over `tracking_time`.
  BackCalculation
}

///|
/// An inclusive band `[low, high]` for one scalar.
#derive(Debug)
pub(all) struct Output {
  low : Double
  high : Double
}

///|
/// A band that never binds.
pub let output_unbounded : Output = { low: -1.0e15, high: 1.0e15, }

///|
/// Every number a PID uses that is not a gain of the loop itself.
#derive(Debug)
pub(all) struct PidConfig {
  kp : Double
  ki : Double
  kd : Double
  weight_p : Double
  weight_d : Double
  derivative_on_measurement : Bool
  anti_windup : AntiWindup
  tracking_time : Double
  filter : Double
  output : Output
}

///|
/// A PID with no gain and no limit: the preset the other constructors start
/// from.
pub let pid_config : PidConfig = {
  kp: 0.0,
  ki: 0.0,
  kd: 0.0,
  weight_p: 1.0,
  weight_d: 1.0,
  derivative_on_measurement: false,
  anti_windup: AntiWindup::Conditional,
  tracking_time: 0.05,
  filter: 0.0,
  output: output_unbounded,
}

///|
/// A PID configuration from the three gains.
pub fn pid_gains(
  kp~ : Double,
  ki~ : Double = 0.0,
  kd~ : Double = 0.0,
) -> PidConfig raise @linear.Refused {
  abort("moonmcl: pid_gains is not implemented yet")
}

///|
/// A proportional-integral-derivative controller with anti-windup, setpoint
/// weighting and a derivative taken on the measurement.
pub(all) struct Pid {
  config : PidConfig
}

///|
/// A PID from a configuration.
pub fn Pid::new(config~ : PidConfig = pid_config) -> Pid {
  { config, }
}

///|
/// The integrator value, for inspection and for tests.
pub fn Pid::integral(self : Pid) -> Double {
  abort("moonmcl: Pid::integral is not implemented yet")
}

///|
pub impl @contract.Controller for Pid with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: Pid::update is not implemented yet")
}

///|
pub impl @contract.Controller for Pid with fn reset(self) {
  abort("moonmcl: Pid::reset is not implemented yet")
}

///|
pub extend Pid with @contract.Controller::{update, reset}

///|
/// A velocity- and acceleration-feedforward term on a reference trajectory.
#derive(Debug)
pub(all) struct Feedforward {
  velocity : Double
  acceleration : Double
}

///|
/// A feedforward with unit velocity gain and no acceleration gain.
pub let feedforward : Feedforward = { velocity: 1.0, acceleration: 0.0, }

///|
/// A feedforward from its two gains.
pub fn Feedforward::new(
  velocity~ : Double = 1.0,
  acceleration~ : Double = 0.0,
) -> Feedforward raise @linear.Refused {
  abort("moonmcl: Feedforward::new is not implemented yet")
}

///|
/// The command for a reference sample.
pub fn Feedforward::command(
  self : Feedforward,
  desired : @contract.Sample,
) -> Double {
  abort("moonmcl: Feedforward::command is not implemented yet")
}

///|
/// A sum of controllers applied in series, the output of one setting the
/// reference of the next.
pub(all) struct Cascade {
  stages : Array[&@contract.Controller]
}

///|
/// A cascade of the given stages, outermost first.
pub fn Cascade::new(stages~ : Array[&@contract.Controller]) -> Cascade {
  { stages, }
}

///|
pub impl @contract.Controller for Cascade with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: Cascade::update is not implemented yet")
}

///|
pub impl @contract.Controller for Cascade with fn reset(self) {
  abort("moonmcl: Cascade::reset is not implemented yet")
}

///|
pub extend Cascade with @contract.Controller::{update, reset}

///|
/// A controller whose output is clamped into a band.
pub(all) struct Saturating {
  inner : &@contract.Controller
  output : Output
}

///|
/// A saturation wrapper.
pub fn Saturating::new(
  inner~ : &@contract.Controller,
  output~ : Output = output_unbounded,
) -> Saturating {
  { inner, output, }
}

///|
pub impl @contract.Controller for Saturating with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: Saturating::update is not implemented yet")
}

///|
pub impl @contract.Controller for Saturating with fn reset(self) {
  abort("moonmcl: Saturating::reset is not implemented yet")
}

///|
pub extend Saturating with @contract.Controller::{update, reset}

///|
/// A controller whose output may not change faster than `rate` per second.
pub(all) struct RateLimited {
  inner : &@contract.Controller
  rate : Double
  previous : Double
}

///|
/// A rate-limit wrapper. A rate that is not positive is refused.
pub fn RateLimited::new(
  inner~ : &@contract.Controller,
  rate~ : Double,
) -> RateLimited raise @linear.Refused {
  abort("moonmcl: RateLimited::new is not implemented yet")
}

///|
pub impl @contract.Controller for RateLimited with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: RateLimited::update is not implemented yet")
}

///|
pub impl @contract.Controller for RateLimited with fn reset(self) {
  abort("moonmcl: RateLimited::reset is not implemented yet")
}

///|
pub extend RateLimited with @contract.Controller::{update, reset}

///|
/// The solution of a discrete algebraic Riccati equation.
#derive(Debug)
pub(all) struct Riccati {
  gain : @linear.Mat
  cost : @linear.Mat
}

///|
/// The discrete algebraic Riccati equation by value iteration.
pub fn dare(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  iterations~ : Int32 = 200,
  tolerance~ : Double = 1.0e-12,
) -> Riccati raise @linear.Refused {
  abort("moonmcl: dare is not implemented yet")
}

///|
/// A linear-quadratic regulator.
pub(all) struct Lqr {
  gain : @linear.Mat
}

///|
/// The infinite-horizon LQR for a discrete `(a, b)` pair.
pub fn lqr(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  iterations~ : Int32 = 200,
  tolerance~ : Double = 1.0e-12,
) -> Lqr raise @linear.Refused {
  abort("moonmcl: lqr is not implemented yet")
}

///|
pub impl @contract.Controller for Lqr with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: Lqr::update is not implemented yet")
}

///|
pub impl @contract.Controller for Lqr with fn reset(self) {
  abort("moonmcl: Lqr::reset is not implemented yet")
}

///|
pub extend Lqr with @contract.Controller::{update, reset}

///|
/// An LQR with an integral of the tracking error appended to the state, so a
/// constant disturbance leaves no steady-state offset.
pub(all) struct IntegralLqr {
  gain : @linear.Mat
  state_dim : Int32
  integral : @linear.Vec
}

///|
/// The integral LQR for a discrete `(a, b)` pair.
pub fn integral_lqr(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  integral_weight~ : @linear.Mat,
  iterations~ : Int32 = 200,
  tolerance~ : Double = 1.0e-12,
) -> IntegralLqr raise @linear.Refused {
  abort("moonmcl: integral_lqr is not implemented yet")
}

///|
pub impl @contract.Controller for IntegralLqr with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  abort("moonmcl: IntegralLqr::update is not implemented yet")
}

///|
pub impl @contract.Controller for IntegralLqr with fn reset(self) {
  abort("moonmcl: IntegralLqr::reset is not implemented yet")
}

///|
pub extend IntegralLqr with @contract.Controller::{update, reset}

///|
/// The knobs of the iterative LQR.
#derive(Debug)
pub(all) struct IlqrConfig {
  iterations : Int32
  tolerance : Double
  control : Output
  regularisation : Double
}

///|
/// The preset the iterative LQR runs with.
pub let ilqr_config : IlqrConfig = {
  iterations: 60,
  tolerance: 1.0e-6,
  control: output_unbounded,
  regularisation: 1.0e-6,
}

///|
/// The input sequence the iterative LQR settles on for a horizon.
pub fn ilqr(
  model~ : &@contract.Model,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
  horizon~ : Int32,
  dt~ : Double,
  config~ : IlqrConfig = ilqr_config,
) -> Array[@linear.Vec] raise @linear.Refused {
  abort("moonmcl: ilqr is not implemented yet")
}

///|
/// The knobs of the model-predictive controller.
#derive(Debug)
pub(all) struct MpcConfig {
  horizon : Int32
  iterations : Int32
  penalty : Double
  input : Output
  rate : Output
}

///|
/// The preset the model-predictive controller runs with.
pub let mpc_config : MpcConfig = {
  horizon: 20,
  iterations: 600,
  penalty: 2.0,
  input: output_unbounded,
  rate: output_unbounded,
}

///|
/// A receding-horizon controller for a linear plant with box limits on the
/// input and on its rate of change.
pub(all) struct Mpc {
  model : @contract.StateSpace
  state_weight : @linear.Mat
  input_weight : @linear.Mat
  config : MpcConfig
  previous : @linear.Vec
}

///|
/// A model-predictive controller for `model` at the given weights.
pub fn Mpc::new(
  model~ : @contract.StateSpace,
  state_weight~ : @linear.Mat,
  input_weight~ : @linear.Mat,
  config~ : MpcConfig = mpc_config,
) -> Mpc raise @linear.Refused {
  abort("moonmcl: Mpc::new is not implemented yet")
}

///|
/// The input for this sample, and the shift of the horizon.
pub fn Mpc::control(
  self : Mpc,
  state~ : @linear.Vec,
  target~ : @linear.Vec,
) -> @linear.Vec raise @linear.Refused {
  abort("moonmcl: Mpc::control is not implemented yet")
}

///|
/// The knobs of the box-constrained quadratic program.
#derive(Debug)
pub(all) struct QpConfig {
  iterations : Int32
  penalty : Double
  tolerance : Double
}

///|
/// The preset the quadratic program runs with.
pub let qp_config : QpConfig = {
  iterations: 600,
  penalty: 2.0,
  tolerance: 1.0e-8,
}

///|
/// The minimiser of `0.5 z' h z + f' z` inside a box.
pub fn box_qp(
  h~ : @linear.Mat,
  f~ : @linear.Vec,
  lower~ : @linear.Vec,
  upper~ : @linear.Vec,
  config~ : QpConfig = qp_config,
) -> @linear.Vec raise @linear.Refused {
  abort("moonmcl: box_qp is not implemented yet")
}

///|
/// A pure-pursuit path follower.
#derive(Debug)
pub(all) struct Pursuit {
  lookahead : Double
  wheelbase : Double
}

///|
/// The preset pure-pursuit follower.
pub let pursuit : Pursuit = { lookahead: 1.0, wheelbase: 1.0, }

///|
/// The steering angle that puts the vehicle on the lookahead point.
pub fn Pursuit::steering(
  self : Pursuit,
  pose~ : @linear.Vec,
  path~ : ArrayView[@linear.Vec],
) -> Double raise @linear.Refused {
  abort("moonmcl: Pursuit::steering is not implemented yet")
}

///|
/// A Stanley path follower.
#derive(Debug)
pub(all) struct Stanley {
  gain : Double
  wheelbase : Double
  soft : Double
}

///|
/// The preset Stanley follower.
pub let stanley : Stanley = { gain: 1.5, wheelbase: 1.0, soft: 0.1, }

///|
/// The steering angle from the heading error and the cross-track error.
pub fn Stanley::steering(
  self : Stanley,
  pose~ : @linear.Vec,
  path~ : ArrayView[@linear.Vec],
) -> Double raise @linear.Refused {
  abort("moonmcl: Stanley::steering is not implemented yet")
}

///|
/// A computed-torque law: the inverse dynamics of a mechanism plus a
/// proportional-derivative correction in joint space.
pub(all) struct ComputedTorque {
  inertia : (@linear.Vec) -> @linear.Mat
  gravity : (@linear.Vec) -> @linear.Vec
  kp : Double
  kd : Double
}

///|
/// The effort that tracks the desired joint motion.
pub fn ComputedTorque::effort(
  self : ComputedTorque,
  q~ : @linear.Vec,
  qd~ : @linear.Vec,
  q_desired~ : @linear.Vec,
  qd_desired~ : @linear.Vec,
  qdd_desired~ : @linear.Vec,
) -> @linear.Vec raise @linear.Refused {
  abort("moonmcl: ComputedTorque::effort is not implemented yet")
}

///|
/// An impedance controller: a mass-spring-damper between the tool and the
/// target, with the contact force measured rather than modelled.
#derive(Debug)
pub(all) struct Impedance {
  mass : Double
  damping : Double
  stiffness : Double
  position : @linear.Vec
  velocity : @linear.Vec
}

///|
/// An impedance from its three parameters and the starting position.
pub fn Impedance::new(
  mass~ : Double,
  damping~ : Double,
  stiffness~ : Double,
  start~ : @linear.Vec,
) -> Impedance raise @linear.Refused {
  abort("moonmcl: Impedance::new is not implemented yet")
}

///|
/// The wrench to command for one period.
pub fn Impedance::effort(
  self : Impedance,
  target~ : @linear.Vec,
  external~ : @linear.Vec,
  dt~ : Double,
) -> @linear.Vec {
  abort("moonmcl: Impedance::effort is not implemented yet")
}

///|
/// The knobs of the control-barrier-function filter.
#derive(Debug)
pub(all) struct CbfConfig {
  iterations : Int32
  tolerance : Double
}

///|
/// The preset the barrier filter runs with.
pub let cbf_config : CbfConfig = { iterations: 64, tolerance: 1.0e-9, }

///|
/// The safe input closest to the nominal one: the projection of `nominal` onto
/// the box and onto the half-space where the barrier's derivative is positive.
pub fn cbf_filter(
  nominal~ : @linear.Vec,
  gradient~ : @linear.Vec,
  drift~ : Double,
  barrier~ : Double,
  alpha~ : Double,
  lower~ : @linear.Vec,
  upper~ : @linear.Vec,
  config~ : CbfConfig = cbf_config,
) -> @linear.Vec {
  abort("moonmcl: cbf_filter is not implemented yet")
}

///|
pub extend AntiWindup with @debug.Debug::{to_repr}

///|
pub extend Output with @debug.Debug::{to_repr}

///|
pub extend PidConfig with @debug.Debug::{to_repr}

///|
pub extend Feedforward with @debug.Debug::{to_repr}

///|
pub extend Riccati with @debug.Debug::{to_repr}

///|
pub extend IlqrConfig with @debug.Debug::{to_repr}

///|
pub extend MpcConfig with @debug.Debug::{to_repr}

///|
pub extend QpConfig with @debug.Debug::{to_repr}

///|
pub extend Pursuit with @debug.Debug::{to_repr}

///|
pub extend Stanley with @debug.Debug::{to_repr}

///|
pub extend Impedance with @debug.Debug::{to_repr}

///|
pub extend CbfConfig with @debug.Debug::{to_repr}
