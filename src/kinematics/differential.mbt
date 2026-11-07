// Differential-drive forward and inverse kinematics, plus a pose update. All
// angles are radians and lengths are metres; the track width is the distance
// between the two drive wheels.

///|
/// A planar pose: position (x, y) and heading theta (radians).
#derive(@debug.Debug)
pub struct Pose {
  // Position along the world x axis, in metres.
  x : Double
  // Position along the world y axis, in metres.
  y : Double
  // Heading, measured anticlockwise from the world x axis.
  theta : Double
}

///|
/// A body-frame twist: linear velocity v along the forward axis and angular
/// velocity w about the normal axis.
#derive(@debug.Debug)
pub struct Twist {
  // Forward velocity, in metres per second.
  v : Double
  // Angular velocity, in radians per second (positive = anticlockwise).
  w : Double
}

///|
/// The two drive wheels' linear velocities (metres per second).
#derive(@debug.Debug)
pub struct WheelVel {
  // Left wheel velocity.
  vl : Double
  // Right wheel velocity.
  vr : Double
}

///|
/// Build a planar pose.
pub fn Pose::pose(x : Double, y : Double, theta : Double) -> Pose {
  { x, y, theta, }
}

///|
/// Build a body-frame twist.
pub fn Twist::twist(v : Double, w : Double) -> Twist {
  { v, w, }
}

///|
/// Build a pair of drive-wheel velocities.
pub fn WheelVel::wheel_vel(vl : Double, vr : Double) -> WheelVel {
  { vl, vr, }
}

///|
pub extend Pose with @debug.Debug::{to_repr}

///|
pub extend Twist with @debug.Debug::{to_repr}

///|
pub extend WheelVel with @debug.Debug::{to_repr}

///|
/// Forward kinematics: the body-frame twist implied by the two wheel
/// velocities over a track width `track_width`.
///   v = (vr + vl) / 2,  w = (vr - vl) / track_width.
pub fn forward_kinematics(wheel : WheelVel, track_width : Double) -> Twist {
  guard track_width > 0.0 else {
    abort("forward_kinematics: track_width must be positive")
  }
  { v: (wheel.vr + wheel.vl) / 2.0, w: (wheel.vr - wheel.vl) / track_width, }
}

///|
/// Inverse kinematics: the two wheel velocities needed to realise the body
/// twist `twist` over a track width `track_width`.
///   vl = v - w * track_width / 2,  vr = v + w * track_width / 2.
pub fn inverse_kinematics(twist : Twist, track_width : Double) -> WheelVel {
  guard track_width > 0.0 else {
    abort("inverse_kinematics: track_width must be positive")
  }
  {
    vl: twist.v - twist.w * track_width / 2.0,
    vr: twist.v + twist.w * track_width / 2.0,
  }
}

///|
/// Propagate a pose by `dt` seconds under a constant body twist (the
/// kinematic bicycle update for a differential drive):
///   x' = x + v dt cos(theta), y' = y + v dt sin(theta), theta' = theta + w dt.
pub fn update_pose(pose : Pose, twist : Twist, dt : Double) -> Pose {
  {
    x: pose.x + twist.v * dt * @math.cos(pose.theta),
    y: pose.y + twist.v * dt * @math.sin(pose.theta),
    theta: pose.theta + twist.w * dt,
  }
}
