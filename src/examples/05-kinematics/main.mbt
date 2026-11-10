// A differential base drawing an arc, and a delta cell answering a pose with its
// three arm angles.
//
//     moon run src/examples/05-kinematics

///|
#coverage.skip
fn main raise {
  let base = @mctl.Differential::new(wheel_radius=0.1, track=0.4)
  let mut pose = @mctl.Vec::of([0.0, 0.0, 0.0])
  // Straight for a second, then a quarter turn on the spot.
  for step in 0..<100 {
    let twist = if step < 50 {
      @mctl.Vec::of([0.5, 0.0])
    } else {
      @mctl.Vec::of([0.0, 1.0])
    }
    pose = base.advance(pose~, twist~, dt=0.02)
  }
  println("x=" + pose.get(0).to_string() + "  y=" + pose.get(1).to_string())
  println("heading=" + pose.get(2).to_string())

  let cell = @mctl.Delta::new(
    base_radius=0.5,
    platform_radius=0.1,
    upper_arm=0.4,
    lower_arm=0.8,
  )
  let target = @mctl.Vec::of([0.05, -0.03, -1.0])
  let joint = cell.inverse(target)
  println("arms " + joint.to_string())
  println("back " + cell.forward(joint).to_string())
}
