// Feedforward control. A model-based pre-compensation that estimates the
// control input needed to follow a reference, independent of the feedback
// loop. The two common forms are a static (gain) feedforward and a
// velocity-compensating (derivative) feedforward; the sum of the two is the
// usual "static + derivative" feedforward used in servo motion.

///|
/// A static + derivative feedforward compensator.
pub struct Feedforward {
  // Static (zero-frequency) gain: the input needed to hold a unit reference.
  kf : Double
  // Derivative (velocity) gain: the input needed to track a unit-rate change.
  kd : Double
}

///|
/// Build a feedforward compensator from its static and derivative gains.
pub fn Feedforward::feedforward(
  kf~ : Double? = Some(1.0),
  kd~ : Double? = Some(0.0),
) -> Feedforward {
  { kf: kf.unwrap_or(1.0), kd: kd.unwrap_or(0.0), }
}

///|
/// The feedforward input for reference `reference` and reference rate
/// `rate`: `kf * reference + kd * rate`.
pub fn Feedforward::of(
  self : Feedforward,
  reference : Double,
  rate : Double,
) -> Double {
  self.kf * reference + self.kd * rate
}
