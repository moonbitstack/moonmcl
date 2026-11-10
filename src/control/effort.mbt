// Two model-based laws: computed torque, which cancels the mechanism's own
// dynamics and then closes a joint-space loop on what is left, and impedance
// control, which gives the tool a mass-spring-damper behaviour instead of a
// position.

///|
/// A computed-torque law: the inertia and gravity of the mechanism plus a
/// proportional-derivative correction in joint space.
///
/// `inertia` and `gravity` are supplied by the caller because the mass matrix of
/// a mechanism belongs to the mechanism, not to the controller; a model that has
/// them can pass them straight in, and a model that does not can pass an identity
/// and get plain joint-space feedback.
pub(all) struct ComputedTorque {
  inertia : (@linear.Vec) -> @linear.Mat
  gravity : (@linear.Vec) -> @linear.Vec
  kp : Double
  kd : Double
}

///|
/// A computed-torque law with the given gains.
pub fn ComputedTorque::new(
  inertia~ : (@linear.Vec) -> @linear.Mat,
  gravity~ : (@linear.Vec) -> @linear.Vec,
  kp~ : Double = 1.0,
  kd~ : Double = 1.0,
) -> ComputedTorque raise @linear.Refused {
  if kp < 0.0 {
    raise @linear.Refused::Gain(got=kp)
  }
  if kd < 0.0 {
    raise @linear.Refused::Gain(got=kd)
  }
  { inertia, gravity, kp, kd, }
}

///|
/// The effort that tracks the desired joint motion.
pub fn ComputedTorque::effort(
  self : ComputedTorque,
  q~ : @linear.Vec,
  qd~ : @linear.Vec,
  q_desired~ : @linear.Vec,
  qd_desired~ : @linear.Vec,
  qdd_desired~ : @linear.Vec,
) -> @linear.Vec raise @linear.Refused {
  let n = q.len()
  if q_desired.len() != n ||
    qd.len() != n ||
    qd_desired.len() != n ||
    qdd_desired.len() != n {
    raise @linear.Refused::Dimension(got=q_desired.len(), want=n)
  }
  let error = q_desired.sub(q)
  let rate = qd_desired.sub(qd)
  let acceleration = qdd_desired
    .add(error.scale(self.kp))
    .add(rate.scale(self.kd))
  (self.inertia)(q).apply(acceleration).add((self.gravity)(q))
}

///|
/// The inertia alone, for a caller that wants the model part without the loop.
pub fn ComputedTorque::inertia_of(
  self : ComputedTorque,
  q~ : @linear.Vec,
) -> @linear.Mat {
  (self.inertia)(q)
}

///|
impl @debug.Debug for ComputedTorque with fn to_repr(self) {
  @debug.Repr::literal(
    "ComputedTorque(kp: " +
    self.kp.to_string() +
    ", kd: " +
    self.kd.to_string() +
    ")",
  )
}

///|
pub extend ComputedTorque with @debug.Debug::{to_repr}

///|
/// An impedance controller: a mass-spring-damper between the tool and the
/// target, with the contact force measured rather than modelled.
///
/// The rendered position follows the admittance `m x'' = f_ext - d x' - k (x - x_d)`
/// and the effort returned is the spring-damper wrench at that position, so a
/// tool pushed off its target yields and then returns.
#derive(Debug)
pub(all) struct Impedance {
  mass : Double
  damping : Double
  stiffness : Double
  mut position : @linear.Vec
  mut velocity : @linear.Vec
}

///|
/// An impedance from its three parameters and the starting position.
pub fn Impedance::new(
  mass~ : Double,
  damping~ : Double,
  stiffness~ : Double,
  start~ : @linear.Vec,
) -> Impedance raise @linear.Refused {
  if mass <= 0.0 {
    raise @linear.Refused::Gain(got=mass)
  }
  if damping < 0.0 || stiffness < 0.0 {
    raise @linear.Refused::Gain(
      got=if damping < 0.0 { damping } else { stiffness },
    )
  }
  {
    mass,
    damping,
    stiffness,
    position: start.clone(),
    velocity: @linear.Vec::zeros(start.len()),
  }
}

///|
/// The rendered position, which is where the tool actually is under a contact
/// force.
pub fn Impedance::position(self : Impedance) -> @linear.Vec {
  self.position
}

///|
/// The wrench to command for one period.
pub fn Impedance::effort(
  self : Impedance,
  target~ : @linear.Vec,
  external~ : @linear.Vec,
  dt~ : Double,
) -> @linear.Vec {
  let n = self.position.len()
  let spring = target.sub(self.position).scale(self.stiffness)
  let damper = self.velocity.scale(-self.damping)
  let force = @linear.Vec::zeros(n)
  for i in 0..<n {
    force.set(i, spring.get(i) + damper.get(i))
  }
  let acceleration = @linear.Vec::zeros(n)
  for i in 0..<n {
    acceleration.set(
      i,
      (external.get(i) + spring.get(i) + damper.get(i)) / self.mass,
    )
  }
  for i in 0..<n {
    self.velocity.set(i, self.velocity.get(i) + acceleration.get(i) * dt)
    self.position.set(i, self.position.get(i) + self.velocity.get(i) * dt)
  }
  force
}

///|
pub extend Impedance with @debug.Debug::{to_repr}
