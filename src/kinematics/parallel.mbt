// The structures that are not a chain: a delta cell, whose platform is held by
// three arms at once, and a gantry, whose beam is driven from both ends.

///|
/// A delta robot: three parallel arms from a fixed base to a moving platform.
///
/// The joint space is the three upper-arm angles measured from horizontal and the
/// task space the platform centre `(x, y, z)` under the base.
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
  if base_radius <= 0.0 || platform_radius <= 0.0 {
    raise @linear.Refused::Gain(got=base_radius)
  }
  if upper_arm <= 0.0 || lower_arm <= 0.0 {
    raise @linear.Refused::Gain(got=upper_arm)
  }
  { base_radius, platform_radius, upper_arm, lower_arm, }
}

///|
/// The elbow of arm `index` for the upper-arm angle `angle`.
fn Delta::elbow(
  self : Delta,
  index : Int32,
  angle : Double,
) -> (Double, Double, Double) {
  let phi = index.to_double() * 2.0943951023931953
  let radial = self.base_radius + self.upper_arm * @math.cos(angle)
  (
    radial * @math.cos(phi),
    radial * @math.sin(phi),
    self.upper_arm * @math.sin(angle),
  )
}

///|
pub impl @contract.Kinematics for Delta with fn forward(self, q) {
  // Each arm holds the platform joint at its own radius, so the centre is the
  // point that is one lower-arm away from all three elbows; subtracting the
  // sphere equations leaves two planes, and the third is a quadratic in z.
  let platform = self.platform_radius
  let arms = 3
  let wx = Array::make(arms, 0.0)
  let wy = Array::make(arms, 0.0)
  let wz = Array::make(arms, 0.0)
  for i in 0..<arms {
    let phi = i.to_double() * 2.0943951023931953
    let (ex, ey, ez) = self.elbow(i, q.get(i))
    wx[i] = ex - platform * @math.cos(phi)
    wy[i] = ey - platform * @math.sin(phi)
    wz[i] = ez
  }
  let a1x = wx[0] - wx[1]
  let a1y = wy[0] - wy[1]
  let a1z = wz[0] - wz[1]
  let a2x = wx[0] - wx[2]
  let a2y = wy[0] - wy[2]
  let a2z = wz[0] - wz[2]
  let norm0 = wx[0] * wx[0] + wy[0] * wy[0] + wz[0] * wz[0]
  let c1 = (norm0 - (wx[1] * wx[1] + wy[1] * wy[1] + wz[1] * wz[1])) / 2.0
  let c2 = (norm0 - (wx[2] * wx[2] + wy[2] * wy[2] + wz[2] * wz[2])) / 2.0
  // x and y as affine functions of z.
  let determinant = a1x * a2y - a1y * a2x
  if determinant.abs() < 1.0e-12 {
    return @linear.Vec::of([0.0, 0.0, 0.0])
  }
  let alpha = (c1 * a2y - c2 * a1y) / determinant
  let beta = (a1y * a2z - a2y * a1z) / determinant
  let gamma = (a1x * c2 - a2x * c1) / determinant
  let delta = (a2x * a1z - a1x * a2z) / determinant
  let ax = beta * beta + delta * delta + 1.0
  let bx = 2.0 * (beta * (alpha - wx[0]) + delta * (gamma - wy[0]) - wz[0])
  let cx = (alpha - wx[0]) * (alpha - wx[0]) +
    (gamma - wy[0]) * (gamma - wy[0]) +
    wz[0] * wz[0] -
    self.lower_arm * self.lower_arm
  let discriminant = bx * bx - 4.0 * ax * cx
  if discriminant < 0.0 {
    return @linear.Vec::of([alpha, gamma, 0.0])
  }
  let z = (-bx - discriminant.sqrt()) / (2.0 * ax)
  @linear.Vec::of([alpha + beta * z, gamma + delta * z, z])
}

///|
/// The arm angles that put the platform centre on `x`, by damped least squares on
/// the forward map.
///
/// The closed form for one arm is a two-link solve in the arm's own plane, but
/// the platform joint is not in that plane unless the centre lies on the arm's
/// radius, so a cell with three arms has no per-arm closed form. Iterating on the
/// forward map instead is exact to the tolerance and costs a handful of Jacobian
/// evaluations.
pub impl @contract.Kinematics for Delta with fn inverse(self, x, seed?) {
  if x.len() != 3 {
    raise @linear.Refused::Dimension(got=x.len(), want=3)
  }
  let joint = match seed {
    Some(value) => value.clone()
    None => {
      // Start from the per-arm two-link answer projected into the arm plane: it
      // ignores the sideways offset, which the iteration then takes out.
      let guess = @linear.Vec::zeros(3)
      for i in 0..<3 {
        let phi = i.to_double() * 2.0943951023931953
        let radial = x.get(0) * @math.cos(phi) +
          x.get(1) * @math.sin(phi) +
          self.platform_radius
        guess.set(i, @math.atan2(x.get(2), radial - self.base_radius))
      }
      guess
    }
  }
  if joint.len() != 3 {
    raise @linear.Refused::Dimension(got=joint.len(), want=3)
  }
  let damping = 0.05
  for _ in 0..<200 {
    let reached = self.forward(joint)
    let error = x.sub(reached)
    if error.norm() <= 1.0e-10 {
      return joint
    }
    let jacobian = self.jacobian(joint)
    let system = jacobian
      .mul(jacobian.transpose())
      .add(@linear.Mat::identity(3).scale(damping * damping))
    let step = jacobian.transpose().apply(@linear.solve_vec(system, error)) catch {
        _ => raise @linear.Refused::Unreachable
      }
    for i in 0..<3 {
      joint.set(i, joint.get(i) + step.get(i))
    }
  }
  let residual = x.sub(self.forward(joint)).norm()
  if residual > 1.0e-6 {
    raise @linear.Refused::Unreachable
  }
  joint
}

///|
pub extend Delta with @contract.Kinematics::{forward, inverse, jacobian}

///|
/// A gantry: two motors drive one gantry axis from either end, so the pair sets
/// the position and the tilt of the beam.
///
/// The joint space is `(x, y_left, y_right, z)` and the task space
/// `(x, y, z, tilt)`, where `y` is the middle of the beam and `tilt` the angle the
/// difference between its ends makes.
#derive(Debug)
pub(all) struct Gantry {
  track : Double
}

///|
/// A gantry with the given beam length.
pub fn Gantry::new(track~ : Double) -> Gantry raise @linear.Refused {
  if track <= 0.0 {
    raise @linear.Refused::Gain(got=track)
  }
  { track, }
}

///|
pub impl @contract.Kinematics for Gantry with fn forward(self, q) {
  let left = q.get(1)
  let right = q.get(2)
  @linear.Vec::of([
    q.get(0),
    (left + right) / 2.0,
    q.get(3),
    (right - left) / self.track,
  ])
}

///|
pub impl @contract.Kinematics for Gantry with fn inverse(self, x, seed?) {
  ignore(seed)
  let centre = x.get(1)
  let tilt = x.get(3)
  @linear.Vec::of([
    x.get(0),
    centre - tilt * self.track / 2.0,
    centre + tilt * self.track / 2.0,
    x.get(2),
  ])
}

///|
pub extend Gantry with @contract.Kinematics::{forward, inverse, jacobian}

///|
pub extend Delta with @debug.Debug::{to_repr}

///|
pub extend Gantry with @debug.Debug::{to_repr}
