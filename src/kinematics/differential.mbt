// The two-wheel base. Its joint space is the two wheel angular velocities and its
// task space the body twist, which is what a motor drive actually commands; the
// pose follows by integrating the twist, and `advance` does that along the arc
// rather than in straight steps.

///|
/// A two-wheel differential drive.
///
/// `track` is the distance between the wheels and `wheel_radius` the radius of
/// each. The joint space is `(omega_left, omega_right)` in radians per second and
/// the task space `(v, omega)` in metres and radians per second.
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
  if wheel_radius <= 0.0 {
    raise @linear.Refused::Gain(got=wheel_radius)
  }
  if track <= 0.0 {
    raise @linear.Refused::Gain(got=track)
  }
  { wheel_radius, track, }
}

///|
pub impl @contract.Kinematics for Differential with fn forward(self, q) {
  let left = q.get(0)
  let right = q.get(1)
  @linear.Vec::of([
    self.wheel_radius * (left + right) / 2.0,
    self.wheel_radius * (right - left) / self.track,
  ])
}

///|
pub impl @contract.Kinematics for Differential with fn inverse(self, x, seed?) {
  ignore(seed)
  let velocity = x.get(0)
  let rotation = x.get(1)
  @linear.Vec::of([
    (velocity - rotation * self.track / 2.0) / self.wheel_radius,
    (velocity + rotation * self.track / 2.0) / self.wheel_radius,
  ])
}

///|
pub extend Differential with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The pose after driving `twist` for `dt` seconds.
///
/// The arc is integrated exactly: a straight chord would cut the corner of every
/// turn, and the error grows with the square of the period.
pub fn Differential::advance(
  self : Differential,
  pose~ : @linear.Vec,
  twist~ : @linear.Vec,
  dt~ : Double,
) -> @linear.Vec {
  ignore(self)
  let velocity = twist.get(0)
  let rotation = twist.get(1)
  let heading = pose.get(2)
  if rotation.abs() < 1.0e-12 {
    @linear.Vec::of([
      pose.get(0) + velocity * dt * @math.cos(heading),
      pose.get(1) + velocity * dt * @math.sin(heading),
      heading,
    ])
  } else {
    let radius = velocity / rotation
    let next = heading + rotation * dt
    @linear.Vec::of([
      pose.get(0) + radius * (@math.sin(next) - @math.sin(heading)),
      pose.get(1) - radius * (@math.cos(next) - @math.cos(heading)),
      next,
    ])
  }
}

///|
pub extend Differential with @debug.Debug::{to_repr}
