// Tests for the kinematics module: the differential-drive forward/inverse
// kinematics round trip and the pose update on a straight line and a heading
// change.

///|
test "forward and inverse kinematics are inverses" {
  let track = 0.5
  let twist = @kinematics.Twist::twist(1.0, 0.5)
  let wheel = @kinematics.inverse_kinematics(twist, track)
  let back = @kinematics.forward_kinematics(wheel, track)
  assert_eq((back.v - 1.0).abs() < 1.0e-12, true)
  assert_eq((back.w - 0.5).abs() < 1.0e-12, true)
}

///|
test "forward kinematics of a pure rotation" {
  // Equal and opposite wheel velocities about a track give zero forward speed
  // and w = 2 * v_wheel / track.
  let track = 0.5
  let wheel = @kinematics.WheelVel::wheel_vel(-1.0, 1.0)
  let twist = @kinematics.forward_kinematics(wheel, track)
  assert_eq(twist.v.abs() < 1.0e-12, true)
  assert_eq((twist.w - 4.0).abs() < 1.0e-12, true)
}

///|
test "pose update on a straight line" {
  let pose = @kinematics.Pose::pose(0.0, 0.0, 0.0)
  let twist = @kinematics.Twist::twist(1.0, 0.0)
  let next = @kinematics.update_pose(pose, twist, 0.5)
  assert_eq((next.x - 0.5).abs() < 1.0e-12, true)
  assert_eq(next.y, 0.0)
  assert_eq(next.theta, 0.0)
}

///|
test "pose update respects the heading" {
  // A heading of pi/2 with forward speed 1.0 moves purely along +y.
  let pose_up = @kinematics.Pose::pose(0.0, 0.0, @math.PI / 2.0)
  let twist = @kinematics.Twist::twist(1.0, 0.0)
  let next = @kinematics.update_pose(pose_up, twist, 0.5)
  assert_eq(next.x.abs() < 1.0e-12, true)
  assert_eq((next.y - 0.5).abs() < 1.0e-12, true)
  assert_eq(next.theta, @math.PI / 2.0)
}

///|
test "pose update rotates the heading" {
  let pose = @kinematics.Pose::pose(0.0, 0.0, 0.0)
  let twist = @kinematics.Twist::twist(0.0, 1.0)
  let next = @kinematics.update_pose(pose, twist, 0.5)
  assert_eq((next.theta - 0.5).abs() < 1.0e-12, true)
  // Pure rotation in place: no translation.
  assert_eq(next.x.abs() < 1.0e-12, true)
  assert_eq(next.y.abs() < 1.0e-12, true)
}
