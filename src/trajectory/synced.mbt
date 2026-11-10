// Multi-axis motion: every axis has its own duration, and the shortest ones are
// slowed so that all of them finish together. A coordinated move that let one
// axis arrive early would draw a different path than the one that was planned.

///|
/// Several single-axis profiles run together.
///
/// Axes are slowed by sampling them on a stretched clock: axis `i` is sampled at
/// `t * duration_i / duration`, and its velocity and acceleration are scaled by
/// the same factor and its square. The path is preserved, the timing is not.
pub(all) struct Synced {
  axes : Array[&@contract.Trajectory]
  duration : Double
}

///|
/// A synchronised set from axes that already have a duration.
pub fn Synced::new(
  axes~ : Array[&@contract.Trajectory],
) -> Synced raise @linear.Refused {
  if axes.length() == 0 {
    raise @linear.Refused::Empty
  }
  let mut longest = 0.0
  for i in 0..<axes.length() {
    let duration = axes[i].duration()
    if duration > longest {
      longest = duration
    }
  }
  if longest <= 0.0 {
    raise @linear.Refused::Unreachable
  }
  { axes, duration: longest, }
}

///|
/// The stretch factor of axis `index`.
fn Synced::stretch(self : Synced, index : Int32) -> Double {
  self.axes[index].duration() / self.duration
}

///|
/// One axis sampled on the common clock, with its derivatives scaled back to the
/// axis's own time.
pub fn Synced::axis_sample(
  self : Synced,
  index : Int32,
  t : Double,
) -> @contract.Sample {
  let factor = self.stretch(index)
  let sample = self.axes[index].sample(t * factor)
  @contract.Sample::new(
    position=sample.position,
    velocity=sample.velocity * factor,
    acceleration=sample.acceleration * factor * factor,
    jerk=sample.jerk * factor * factor * factor,
  )
}

///|
/// Every axis sampled at the same instant.
pub fn Synced::sample_all(self : Synced, t : Double) -> Array[@contract.Sample] {
  let out = Array::make(self.axes.length(), @contract.Sample::still(0.0))
  for i in 0..<self.axes.length() {
    out[i] = self.axis_sample(i, t)
  }
  out
}

///|
pub impl @contract.Trajectory for Synced with fn duration(self) {
  self.duration
}

///|
pub impl @contract.Trajectory for Synced with fn sample(self, t) {
  // A synchronised set is not a single axis; the first axis is the one a caller
  // that treats it as one gets, and `sample_all` is the honest interface.
  self.axis_sample(0, t)
}

///|
pub extend Synced with @contract.Trajectory::{duration, sample}

///|
impl @debug.Debug for Synced with fn to_repr(self) {
  @debug.Repr::literal("Synced(" + self.axes.length().to_string() + " axes)")
}

///|
pub extend Synced with @debug.Debug::{to_repr}
