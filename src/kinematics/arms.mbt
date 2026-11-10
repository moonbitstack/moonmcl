// The serial arms. The planar chain is the one that shows the whole method: an
// analytic Jacobian, and an inverse by damped least squares, which is what keeps
// the solve finite at a singularity instead of asking for an unbounded torque.

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
  if lengths.length() == 0 {
    raise @linear.Refused::Empty
  }
  for i in 0..<lengths.length() {
    if lengths[i] <= 0.0 {
      raise @linear.Refused::Gain(got=lengths[i])
    }
  }
  let copied = Array::make(lengths.length(), 0.0)
  for i in 0..<lengths.length() {
    copied[i] = lengths[i]
  }
  { lengths: copied, }
}

///|
pub impl @contract.Kinematics for PlanarArm with fn forward(self, q) {
  let mut x = 0.0
  let mut y = 0.0
  let mut angle = 0.0
  for i in 0..<self.lengths.length() {
    angle = angle + q.get(i)
    x = x + self.lengths[i] * @math.cos(angle)
    y = y + self.lengths[i] * @math.sin(angle)
  }
  @linear.Vec::of([x, y])
}

///|
pub impl @contract.Kinematics for PlanarArm with fn inverse(self, x, seed?) {
  let start = match seed {
    Some(value) => value
    None => @linear.Vec::zeros(self.lengths.length())
  }
  self.solve(target=x, seed=start)
}

///|
pub fn PlanarArm::jacobian_analytic(
  self : PlanarArm,
  q : @linear.Vec,
) -> @linear.Mat {
  let n = self.lengths.length()
  let jacobian = @linear.Mat::zeros(2, n)
  for k in 0..<n {
    let mut x = 0.0
    let mut y = 0.0
    // The links from k outwards move, and each one sits at the sum of every joint
    // up to it, including the ones before k.
    let mut angle = 0.0
    for i in 0..<k {
      angle = angle + q.get(i)
    }
    for i in k..<n {
      angle = angle + q.get(i)
      x = x - self.lengths[i] * @math.sin(angle)
      y = y + self.lengths[i] * @math.cos(angle)
    }
    jacobian.set(0, k, x)
    jacobian.set(1, k, y)
  }
  jacobian
}

///|
/// The joint angles that put the tip on `target`, by damped least squares.
///
/// The damping is what makes the solve usable near a singularity: without it the
/// Jacobian's inverse blows up and the joint velocities with it, which on a real
/// arm is a torque the drive cannot deliver. `damping` is the square of the
/// smallest singular value the solve allows.
pub fn PlanarArm::solve(
  self : PlanarArm,
  target~ : @linear.Vec,
  seed~ : @linear.Vec,
  iterations~ : Int32 = 200,
  damping~ : Double = 0.05,
  tolerance~ : Double = 1.0e-9,
) -> @linear.Vec raise @linear.Refused {
  let n = self.lengths.length()
  if target.len() != 2 {
    raise @linear.Refused::Dimension(got=target.len(), want=2)
  }
  if seed.len() != n {
    raise @linear.Refused::Dimension(got=seed.len(), want=n)
  }
  let joint = seed.clone()
  let squared = damping * damping
  for _ in 0..<iterations {
    let tip = self.forward(joint)
    let error = target.sub(tip)
    if error.norm() <= tolerance {
      return joint
    }
    let jacobian = self.jacobian_analytic(joint)
    // dq = j' (j j' + lambda^2 i)^-1 e
    let system = jacobian
      .mul(jacobian.transpose())
      .add(@linear.Mat::identity(2).scale(squared))
    let step = jacobian.transpose().apply(@linear.solve_vec(system, error)) catch {
        _ => raise @linear.Refused::Unreachable
      }
    for i in 0..<n {
      joint.set(i, joint.get(i) + step.get(i))
    }
  }
  let error = target.sub(self.forward(joint)).norm()
  if error > tolerance {
    raise @linear.Refused::Exceeded(limit=iterations, got=iterations)
  }
  joint
}

///|
pub extend PlanarArm with @contract.Kinematics::{forward, inverse, jacobian}

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
  if upper_arm <= 0.0 || forearm <= 0.0 {
    raise @linear.Refused::Gain(got=upper_arm)
  }
  if vertical <= 0.0 {
    raise @linear.Refused::Gain(got=vertical)
  }
  { upper_arm, forearm, vertical, }
}

///|
pub impl @contract.Kinematics for Scara with fn forward(self, q) {
  let first = q.get(0)
  let second = q.get(1)
  let reach = self.upper_arm + self.forearm
  ignore(reach)
  @linear.Vec::of([
    self.upper_arm * @math.cos(first) + self.forearm * @math.cos(first + second),
    self.upper_arm * @math.sin(first) + self.forearm * @math.sin(first + second),
    q.get(2),
  ])
}

///|
pub impl @contract.Kinematics for Scara with fn inverse(self, x, seed?) {
  let px = x.get(0)
  let py = x.get(1)
  let reach = (px * px + py * py).sqrt()
  let longest = self.upper_arm + self.forearm
  let shortest = (self.upper_arm - self.forearm).abs()
  if reach > longest || reach < shortest {
    raise @linear.Refused::Unreachable
  }
  let cosine = (
      reach * reach -
      self.upper_arm * self.upper_arm -
      self.forearm * self.forearm
    ) /
    (2.0 * self.upper_arm * self.forearm)
  let clamped = if cosine > 1.0 {
    1.0
  } else if cosine < -1.0 {
    -1.0
  } else {
    cosine
  }
  let second = match seed {
    // The seed picks the elbow: a positive second joint is one configuration, a
    // negative one the other.
    Some(value) =>
      if value.len() > 1 && value.get(1) < 0.0 {
        -@math.acos(clamped)
      } else {
        @math.acos(clamped)
      }
    None => @math.acos(clamped)
  }
  let first = @math.atan2(py, px) -
    @math.atan2(
      self.forearm * @math.sin(second),
      self.upper_arm + self.forearm * @math.cos(second),
    )
  @linear.Vec::of([first, second, x.get(2)])
}

///|
pub extend Scara with @contract.Kinematics::{forward, inverse, jacobian}

///|
pub extend PlanarArm with @debug.Debug::{to_repr}

///|
pub extend Scara with @debug.Debug::{to_repr}
