// The geometric path followers. Both take a pose `(x, y, heading)` and a path of
// points, and return a steering angle; the speed is the caller's business, since
// a follower that also sets the speed would decide the cycle time of the job.

///|
/// The half turn, to fold an angle into `(-pi, pi]`.
let half_turn : Double = 3.141592653589793

///|
/// The steering a pure-pursuit follower asks for.
///
/// The follower looks a fixed distance ahead on the path, which is what makes it
/// stable at speed: a short lookahead cuts corners and oscillates, a long one
/// turns early. The returned angle is the one that puts the rear axle on an arc
/// through the lookahead point.
#derive(Debug)
pub(all) struct Pursuit {
  lookahead : Double
  wheelbase : Double
}

///|
/// The preset pure-pursuit follower: one metre of lookahead on a one-metre
/// wheelbase.
pub let pursuit : Pursuit = { lookahead: 1.0, wheelbase: 1.0, }

///|
/// The steering angle that puts the vehicle on the lookahead point.
pub fn Pursuit::steering(
  self : Pursuit,
  pose~ : @linear.Vec,
  path~ : ArrayView[@linear.Vec],
) -> Double raise @linear.Refused {
  if pose.len() < 3 {
    raise @linear.Refused::Dimension(got=pose.len(), want=3)
  }
  if path.length() < 2 {
    raise @linear.Refused::Dimension(got=path.length(), want=2)
  }
  if self.lookahead <= 0.0 || self.wheelbase <= 0.0 {
    raise @linear.Refused::Gain(got=self.lookahead)
  }
  let target = lookahead_point(pose, path, self.lookahead)
  let dx = target.get(0) - pose.get(0)
  let dy = target.get(1) - pose.get(1)
  let alpha = @math.atan2(dy, dx) - pose.get(2)
  let distance = (dx * dx + dy * dy).sqrt()
  if distance == 0.0 {
    return 0.0
  }
  @math.atan2(2.0 * self.wheelbase * @math.sin(alpha), distance)
}

///|
/// The steering a Stanley follower asks for.
///
/// The law is the heading error plus a term that grows with the distance from the
/// path: the first brings the vehicle parallel, the second brings it back on.
/// `soft` stands in for the speed in the usual `atan(k e / v)`, so the follower
/// does not need a speed input.
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
  if pose.len() < 3 {
    raise @linear.Refused::Dimension(got=pose.len(), want=3)
  }
  if path.length() < 2 {
    raise @linear.Refused::Dimension(got=path.length(), want=2)
  }
  if self.soft <= 0.0 {
    raise @linear.Refused::Gain(got=self.soft)
  }
  let nearest = project_pose(pose, path)
  let cross = nearest.cross_track
  let heading = nearest.heading
  let error = fold_angle(heading - pose.get(2))
  let correction = @math.atan2(self.gain * cross, self.soft)
  error + correction
}

///|
/// The point on the path one lookahead away from the pose, measured forward from
/// the projection of the pose onto the path.
fn lookahead_point(
  pose : @linear.Vec,
  path : ArrayView[@linear.Vec],
  lookahead : Double,
) -> @linear.Vec {
  let projection = project_pose(pose, path)
  let mut accumulated = 0.0
  let mut previous = projection.point
  for i in projection.segment..<path.length() {
    let current = path[i]
    let dx = current.get(0) - previous.get(0)
    let dy = current.get(1) - previous.get(1)
    let length = (dx * dx + dy * dy).sqrt()
    if accumulated + length >= lookahead && length > 0.0 {
      let t = (lookahead - accumulated) / length
      return @linear.Vec::of([
        previous.get(0) + t * dx,
        previous.get(1) + t * dy,
      ])
    }
    accumulated = accumulated + length
    previous = current
  }
  previous
}

///|
/// Where the pose projects onto the path: the point, the segment it fell on and
/// how far off the path it is.
priv struct Projection {
  point : @linear.Vec
  segment : Int32
  cross_track : Double
  heading : Double
}

///|
/// The projection of a pose onto the closest segment of the path.
fn project_pose(
  pose : @linear.Vec,
  path : ArrayView[@linear.Vec],
) -> Projection {
  let mut best : Projection = {
    point: path[0],
    segment: 1,
    cross_track: 0.0,
    heading: 0.0,
  }
  let mut best_distance = 1.0e30
  for i in 1..<path.length() {
    let a = path[i - 1]
    let b = path[i]
    let abx = b.get(0) - a.get(0)
    let aby = b.get(1) - a.get(1)
    let length_squared = abx * abx + aby * aby
    let t = if length_squared > 0.0 {
      clamp_scalar(
        ((pose.get(0) - a.get(0)) * abx + (pose.get(1) - a.get(1)) * aby) /
        length_squared,
        0.0,
        1.0,
      )
    } else {
      0.0
    }
    let px = a.get(0) + t * abx
    let py = a.get(1) + t * aby
    let dx = pose.get(0) - px
    let dy = pose.get(1) - py
    let distance = dx * dx + dy * dy
    if distance < best_distance {
      best_distance = distance
      let heading = @math.atan2(aby, abx)
      // Positive when the path lies to the left of the vehicle, which is the
      // sign the Stanley law needs to steer back onto it.
      let signed = @math.sin(heading) * dx - @math.cos(heading) * dy
      best = {
        point: @linear.Vec::of([px, py]),
        segment: if t >= 1.0 {
          i + 1
        } else {
          i
        },
        cross_track: signed,
        heading,
      }
    }
  }
  best
}

///|
/// Fold an angle into `(-pi, pi]`.
fn fold_angle(angle : Double) -> Double {
  let mut value = angle
  while value > half_turn {
    value = value - 2.0 * half_turn
  }
  while value <= -half_turn {
    value = value + 2.0 * half_turn
  }
  value
}

///|
pub extend Pursuit with @debug.Debug::{to_repr}

///|
pub extend Stanley with @debug.Debug::{to_repr}
