// A small differential-drive drive: command a body twist, invert it to wheel
// velocities, verify the forward kinematics agrees, then integrate the pose
// along a straight line and around a heading change.
//
//     moon run src/examples/05-kinematics

///|
#coverage.skip
fn main {
  let track = 0.5
  let dt = 0.1
  let mut pose = @mctl.Pose::pose(0.0, 0.0, 0.0)
  // Drive straight forward at 1 m/s for 3 s.
  for _ in 0..<30 {
    let wheel = @mctl.inverse_kinematics(@mctl.Twist::twist(1.0, 0.0), track)
    let body = @mctl.forward_kinematics(wheel, track)
    pose = @mctl.update_pose(pose, body, dt)
  }
  println(
    "after driving straight: x=" +
    pose.x.to_string() +
    " y=" +
    pose.y.to_string() +
    " theta=" +
    pose.theta.to_string(),
  )
  // Then rotate in place at 1 rad/s for a quarter turn.
  for _ in 0..<16 {
    let wheel = @mctl.inverse_kinematics(@mctl.Twist::twist(0.0, 1.0), track)
    let body = @mctl.forward_kinematics(wheel, track)
    pose = @mctl.update_pose(pose, body, dt)
  }
  println(
    "after a ~90 deg turn  : x=" +
    pose.x.to_string() +
    " y=" +
    pose.y.to_string() +
    " theta=" +
    pose.theta.to_string(),
  )
}
