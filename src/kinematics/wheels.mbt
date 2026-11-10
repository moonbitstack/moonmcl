// The steering bases and the wheels that can push sideways. Each one answers the
// same two questions: what the body does for a set of wheel commands, and what
// commands produce a body motion.

///|
/// A bicycle model: the joint space is `(speed, steering angle)` and the task
/// space the body twist `(vx, vy, omega)`.
#derive(Debug)
pub(all) struct Bicycle {
  wheelbase : Double
}

///|
/// A bicycle with the given wheelbase.
pub fn Bicycle::new(wheelbase~ : Double) -> Bicycle raise @linear.Refused {
  if wheelbase <= 0.0 {
    raise @linear.Refused::Gain(got=wheelbase)
  }
  { wheelbase, }
}

///|
pub impl @contract.Kinematics for Bicycle with fn forward(self, q) {
  let velocity = q.get(0)
  let steering = q.get(1)
  @linear.Vec::of([
    velocity,
    0.0,
    velocity * @math.tan(steering) / self.wheelbase,
  ])
}

///|
pub impl @contract.Kinematics for Bicycle with fn inverse(self, x, seed?) {
  ignore(seed)
  let velocity = x.get(0)
  let rotation = x.get(2)
  let steering = if velocity.abs() < 1.0e-12 {
    0.0
  } else {
    @math.atan(rotation * self.wheelbase / velocity)
  }
  @linear.Vec::of([velocity, steering])
}

///|
pub extend Bicycle with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The curvature a steering angle gives.
pub fn Bicycle::curvature(self : Bicycle, steering~ : Double) -> Double {
  @math.tan(steering) / self.wheelbase
}

///|
/// The steering angle a curvature needs.
pub fn Bicycle::steering(self : Bicycle, curvature~ : Double) -> Double {
  @math.atan(curvature * self.wheelbase)
}

///|
/// An Ackermann steering base: the inner and outer front wheels turn by different
/// angles about a common centre.
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
  if wheelbase <= 0.0 {
    raise @linear.Refused::Gain(got=wheelbase)
  }
  if track <= 0.0 {
    raise @linear.Refused::Gain(got=track)
  }
  { wheelbase, track, }
}

///|
pub impl @contract.Kinematics for Ackermann with fn forward(self, q) {
  let velocity = q.get(0)
  let steering = q.get(1)
  @linear.Vec::of([
    velocity,
    0.0,
    velocity * @math.tan(steering) / self.wheelbase,
  ])
}

///|
pub impl @contract.Kinematics for Ackermann with fn inverse(self, x, seed?) {
  ignore(seed)
  let velocity = x.get(0)
  let rotation = x.get(2)
  let steering = if velocity.abs() < 1.0e-12 {
    0.0
  } else {
    @math.atan(rotation * self.wheelbase / velocity)
  }
  @linear.Vec::of([velocity, steering])
}

///|
pub extend Ackermann with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// The inner and outer steering angles for a curvature: the wheel on the inside
/// of the turn has to turn further, by the angle its shorter radius demands.
pub fn Ackermann::angles(
  self : Ackermann,
  curvature~ : Double,
) -> (Double, Double) {
  if curvature.abs() < 1.0e-12 {
    return (0.0, 0.0)
  }
  let radius = 1.0 / curvature
  let inner = @math.atan(self.wheelbase / (radius.abs() - self.track / 2.0))
  let outer = @math.atan(self.wheelbase / (radius.abs() + self.track / 2.0))
  if curvature > 0.0 {
    (inner, outer)
  } else {
    (-inner, -outer)
  }
}

///|
/// A four-wheel mecanum base. The joint space is the four wheel speeds in the
/// order front left, front right, rear left, rear right, and the task space the
/// body twist `(vx, vy, omega)`.
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
  if wheel_radius <= 0.0 {
    raise @linear.Refused::Gain(got=wheel_radius)
  }
  if half_length <= 0.0 || half_width <= 0.0 {
    raise @linear.Refused::Gain(got=half_length)
  }
  { wheel_radius, half_length, half_width, }
}

///|
pub impl @contract.Kinematics for Mecanum with fn forward(self, q) {
  let radius = self.wheel_radius
  let spread = self.half_length + self.half_width
  @linear.Vec::of([
    radius * (q.get(0) + q.get(1) + q.get(2) + q.get(3)) / 4.0,
    radius * (-q.get(0) + q.get(1) + q.get(2) - q.get(3)) / 4.0,
    radius * (-q.get(0) + q.get(1) - q.get(2) + q.get(3)) / (4.0 * spread),
  ])
}

///|
pub impl @contract.Kinematics for Mecanum with fn inverse(self, x, seed?) {
  ignore(seed)
  let radius = self.wheel_radius
  let spread = self.half_length + self.half_width
  let vx = x.get(0)
  let vy = x.get(1)
  let rotation = x.get(2)
  @linear.Vec::of([
    (vx - vy - spread * rotation) / radius,
    (vx + vy + spread * rotation) / radius,
    (vx + vy - spread * rotation) / radius,
    (vx - vy + spread * rotation) / radius,
  ])
}

///|
pub extend Mecanum with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// An omnidirectional base with three or more driven wheels on a circle.
///
/// The wheels are mounted tangentially, so each one drives along `theta` and the
/// map from twist to wheel speeds is one row per wheel. With three wheels it
/// inverts exactly; with more the inverse is the least-squares one, which spreads
/// the effort over the wheels that can help.
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
  if wheel_radius <= 0.0 {
    raise @linear.Refused::Gain(got=wheel_radius)
  }
  if radius <= 0.0 {
    raise @linear.Refused::Gain(got=radius)
  }
  if angles.length() < 3 {
    raise @linear.Refused::Dimension(got=angles.length(), want=3)
  }
  let copied = Array::make(angles.length(), 0.0)
  for i in 0..<angles.length() {
    copied[i] = angles[i]
  }
  { wheel_radius, radius, angles: copied, }
}

///|
pub impl @contract.Kinematics for Omnidirectional with fn forward(self, q) {
  // The least-squares twist that the wheel speeds best represent: the normal
  // equations of the wheel map, solved for three unknowns.
  let map = self.wheel_map()
  let transpose = map.transpose()
  let normal = transpose.mul(map)
  let right = transpose.apply(q).scale(self.wheel_radius)
  @linear.solve_vec(normal, right) catch {
    _ => @linear.Vec::zeros(3)
  }
}

///|
pub impl @contract.Kinematics for Omnidirectional with fn inverse(
  self,
  x,
  seed?,
) {
  ignore(seed)
  let map = self.wheel_map()
  let wheel_speeds = map.apply(x).scale(1.0 / self.wheel_radius)
  wheel_speeds
}

///|
/// The matrix that turns a twist into the wheel surface speeds, one row per
/// wheel.
fn Omnidirectional::wheel_map(self : Omnidirectional) -> @linear.Mat {
  let count = self.angles.length()
  let map = @linear.Mat::zeros(count, 3)
  for i in 0..<count {
    let angle = self.angles[i]
    map.set(i, 0, @math.cos(angle))
    map.set(i, 1, @math.sin(angle))
    map.set(i, 2, self.radius)
  }
  map
}

///|
pub extend Omnidirectional with @contract.Kinematics::{
  forward,
  inverse,
  jacobian,
}

///|
pub extend Bicycle with @debug.Debug::{to_repr}

///|
pub extend Ackermann with @debug.Debug::{to_repr}

///|
pub extend Mecanum with @debug.Debug::{to_repr}

///|
pub extend Omnidirectional with @debug.Debug::{to_repr}
