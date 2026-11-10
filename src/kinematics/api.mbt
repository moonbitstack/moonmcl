///|
/// A two-wheel differential drive. The joint space is the two wheel angular
/// velocities and the task space the body twist `(v, omega)`.
#derive(Debug)
pub(all) struct Differential {
  wheel_radius : Double
  track : Double
}

///|
/// A differential drive with the given wheel radius and track width.
pub fn Differential::new(
  wheel_radius~ : Double,
  track~ : Double,
) -> Differential raise @linear.Refused {
  abort("moonmcl: Differential::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Differential with fn forward(self, q) {
  abort("moonmcl: Differential::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Differential with fn inverse(self, x, seed?) {
  abort("moonmcl: Differential::inverse is not implemented yet")
}

///|
pub extend Differential with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The pose after driving `twist` for `dt` seconds, integrated along the arc
/// rather than in straight steps.
pub fn Differential::advance(
  self : Differential,
  pose~ : @linear.Vec,
  twist~ : @linear.Vec,
  dt~ : Double,
) -> @linear.Vec {
  abort("moonmcl: Differential::advance is not implemented yet")
}

///|
/// A bicycle model. The joint space is `(speed, steering angle)` and the task
/// space the body twist.
#derive(Debug)
pub(all) struct Bicycle {
  wheelbase : Double
}

///|
/// A bicycle with the given wheelbase.
pub fn Bicycle::new(wheelbase~ : Double) -> Bicycle raise @linear.Refused {
  abort("moonmcl: Bicycle::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Bicycle with fn forward(self, q) {
  abort("moonmcl: Bicycle::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Bicycle with fn inverse(self, x, seed?) {
  abort("moonmcl: Bicycle::inverse is not implemented yet")
}

///|
pub extend Bicycle with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The curvature a steering angle gives.
pub fn Bicycle::curvature(self : Bicycle, steering~ : Double) -> Double {
  abort("moonmcl: Bicycle::curvature is not implemented yet")
}

///|
/// The steering angle a curvature needs.
pub fn Bicycle::steering(self : Bicycle, curvature~ : Double) -> Double {
  abort("moonmcl: Bicycle::steering is not implemented yet")
}

///|
/// An Ackermann steering base: the inner and outer front wheels turn by
/// different angles about a common centre.
#derive(Debug)
pub(all) struct Ackermann {
  wheelbase : Double
  track : Double
}

///|
/// An Ackermann base with the given wheelbase and track.
pub fn Ackermann::new(
  wheelbase~ : Double,
  track~ : Double,
) -> Ackermann raise @linear.Refused {
  abort("moonmcl: Ackermann::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Ackermann with fn forward(self, q) {
  abort("moonmcl: Ackermann::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Ackermann with fn inverse(self, x, seed?) {
  abort("moonmcl: Ackermann::inverse is not implemented yet")
}

///|
pub extend Ackermann with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The inner and outer steering angles for a curvature.
pub fn Ackermann::angles(
  self : Ackermann,
  curvature~ : Double,
) -> (Double, Double) {
  abort("moonmcl: Ackermann::angles is not implemented yet")
}

///|
/// A four-wheel mecanum base. The joint space is the four wheel speeds and the
/// task space the body twist `(vx, vy, omega)`.
#derive(Debug)
pub(all) struct Mecanum {
  wheel_radius : Double
  half_length : Double
  half_width : Double
}

///|
/// A mecanum base with the given wheel radius and half extents.
pub fn Mecanum::new(
  wheel_radius~ : Double,
  half_length~ : Double,
  half_width~ : Double,
) -> Mecanum raise @linear.Refused {
  abort("moonmcl: Mecanum::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Mecanum with fn forward(self, q) {
  abort("moonmcl: Mecanum::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Mecanum with fn inverse(self, x, seed?) {
  abort("moonmcl: Mecanum::inverse is not implemented yet")
}

///|
pub extend Mecanum with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// An omnidirectional base with three or more driven wheels on a circle.
#derive(Debug)
pub(all) struct Omnidirectional {
  wheel_radius : Double
  radius : Double
  angles : Array[Double]
}

///|
/// An omnidirectional base whose wheel `i` sits at `angles[i]` radians on a
/// circle of `radius`.
pub fn Omnidirectional::new(
  wheel_radius~ : Double,
  radius~ : Double,
  angles~ : ArrayView[Double],
) -> Omnidirectional raise @linear.Refused {
  abort("moonmcl: Omnidirectional::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Omnidirectional with fn forward(self, q) {
  abort("moonmcl: Omnidirectional::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Omnidirectional with fn inverse(
  self,
  x,
  seed?,
) {
  abort("moonmcl: Omnidirectional::inverse is not implemented yet")
}

///|
pub extend Omnidirectional with @contract.Kinematics::{
  forward,
  inverse,
  jacobian,
}

///|
/// A planar chain of `n` links. The joint space is the joint angles and the task
/// space the `(x, y)` of the tip.
#derive(Debug)
pub(all) struct PlanarArm {
  lengths : Array[Double]
}

///|
/// A planar arm with the given link lengths.
pub fn PlanarArm::new(
  lengths~ : ArrayView[Double],
) -> PlanarArm raise @linear.Refused {
  abort("moonmcl: PlanarArm::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for PlanarArm with fn forward(self, q) {
  abort("moonmcl: PlanarArm::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for PlanarArm with fn inverse(self, x, seed?) {
  abort("moonmcl: PlanarArm::inverse is not implemented yet")
}

///|
pub fn PlanarArm::jacobian_analytic(
  self : PlanarArm,
  q : @linear.Vec,
) -> @linear.Mat {
  abort("moonmcl: PlanarArm::jacobian_analytic is not implemented yet")
}

///|
pub extend PlanarArm with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The damped-least-squares inverse kinematics of the chain towards `target`.
pub fn PlanarArm::solve(
  self : PlanarArm,
  target~ : @linear.Vec,
  seed~ : @linear.Vec,
  iterations~ : Int32 = 200,
  damping~ : Double = 0.05,
  tolerance~ : Double = 1.0e-9,
) -> @linear.Vec raise @linear.Refused {
  abort("moonmcl: PlanarArm::solve is not implemented yet")
}

///|
/// A delta robot: three parallel arms from a fixed base to a moving platform.
#derive(Debug)
pub(all) struct Delta {
  base_radius : Double
  platform_radius : Double
  upper_arm : Double
  lower_arm : Double
}

///|
/// A delta cell with the given geometry.
pub fn Delta::new(
  base_radius~ : Double,
  platform_radius~ : Double,
  upper_arm~ : Double,
  lower_arm~ : Double,
) -> Delta raise @linear.Refused {
  abort("moonmcl: Delta::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Delta with fn forward(self, q) {
  abort("moonmcl: Delta::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Delta with fn inverse(self, x, seed?) {
  abort("moonmcl: Delta::inverse is not implemented yet")
}

///|
pub extend Delta with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// A SCARA arm: two revolute joints about the vertical axis and one prismatic
/// joint along it.
#derive(Debug)
pub(all) struct Scara {
  upper_arm : Double
  forearm : Double
  vertical : Double
}

///|
/// A SCARA with the two link lengths and the vertical stroke.
pub fn Scara::new(
  upper_arm~ : Double,
  forearm~ : Double,
  vertical~ : Double,
) -> Scara raise @linear.Refused {
  abort("moonmcl: Scara::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Scara with fn forward(self, q) {
  abort("moonmcl: Scara::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Scara with fn inverse(self, x, seed?) {
  abort("moonmcl: Scara::inverse is not implemented yet")
}

///|
pub extend Scara with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// A gantry: two motors drive one gantry axis from either end, so the pair sets
/// the position and the tilt of the beam.
#derive(Debug)
pub(all) struct Gantry {
  track : Double
}

///|
/// A gantry with the given beam length.
pub fn Gantry::new(track~ : Double) -> Gantry raise @linear.Refused {
  abort("moonmcl: Gantry::new is not implemented yet")
}

///|
pub impl @contract.Kinematics for Gantry with fn forward(self, q) {
  abort("moonmcl: Gantry::forward is not implemented yet")
}

///|
pub impl @contract.Kinematics for Gantry with fn inverse(self, x, seed?) {
  abort("moonmcl: Gantry::inverse is not implemented yet")
}

///|
pub extend Gantry with @contract.Kinematics::{forward, inverse, jacobian}

///|
pub extend Differential with @debug.Debug::{to_repr}

///|
pub extend Bicycle with @debug.Debug::{to_repr}

///|
pub extend Ackermann with @debug.Debug::{to_repr}

///|
pub extend Mecanum with @debug.Debug::{to_repr}

///|
pub extend Omnidirectional with @debug.Debug::{to_repr}

///|
pub extend PlanarArm with @debug.Debug::{to_repr}

///|
pub extend Delta with @debug.Debug::{to_repr}

///|
pub extend Scara with @debug.Debug::{to_repr}

///|
pub extend Gantry with @debug.Debug::{to_repr}
