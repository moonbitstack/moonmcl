// Geometric path followers for a unicycle. Both consume a local reference
// point (a "lookahead" or "waypoint") relative to the robot's frame and return
// a steering angle (radians) and a feedforward speed. The reference point is
// the caller's job: a pure-pursuit lookahead or the nearest path sample.

///|
/// The steering angle and speed returned by a path follower.
pub struct Steering {
  // Steering angle in radians (0 = straight, positive = left).
  angle : Double
  // Feedforward forward speed.
  speed : Double
}

///|
/// The reference point in the robot frame, used to compute the lookahead
/// geometry.
pub struct Lookahead {
  // Lateral offset of the lookahead point (positive = left).
  y : Double
  // Distance of the lookahead point ahead of the robot (forward axis).
  l : Double
}

///|
/// Build a lookahead reference from its lateral offset and forward distance.
pub fn Lookahead::lookahead(y : Double, l : Double) -> Lookahead {
  { y, l, }
}

///|
/// Pure pursuit: with a lookahead point at (l, y) ahead and to the left, the
/// heading error is theta = atan2(y, l) and the path curvature is
/// kappa = 2 sin(theta) / l; the front-wheel angle follows the bicycle
/// geometry angle = atan(wheelbase * kappa).
pub fn pure_pursuit(
  wheelbase : Double,
  lookahead : Lookahead,
  speed~ : Double? = Some(1.0),
) -> Steering {
  guard wheelbase > 0.0 else {
    abort("pure_pursuit: wheelbase must be positive")
  }
  guard lookahead.l > 0.0 else {
    abort("pure_pursuit: lookahead distance must be positive")
  }
  let heading_error = @math.atan2(lookahead.y, lookahead.l)
  let v = speed.unwrap_or(1.0)
  let kappa = 2.0 * @math.sin(heading_error) / lookahead.l
  let delta = @math.atan(wheelbase * kappa)
  { angle: delta, speed: v, }
}

///|
/// Stanley: the classic longitudinal + lateral law. With a heading error
/// `heading_error` (robot orientation relative to the path tangent) and a
/// lateral offset `y`, the command is
///   angle = heading_error + atan(-kp * y / max(v, v_min)).
pub fn stanley(
  kp : Double,
  heading_error : Double,
  y : Double,
  v : Double,
  v_min~ : Double? = Some(0.1),
) -> Steering {
  let vmin = v_min.unwrap_or(0.1)
  guard vmin > 0.0 else { abort("stanley: v_min must be positive") }
  let effective = if v.abs() < vmin { vmin } else { v.abs() }
  let lateral = @math.atan(-kp * y / effective)
  { angle: heading_error + lateral, speed: v, }
}
