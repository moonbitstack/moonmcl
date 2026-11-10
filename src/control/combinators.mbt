// The combinators: they take any controller and change how it is driven or how
// its output is allowed to move. Because they only know the `Controller`
// contract, a controller written outside this library composes the same way.

///|
/// Controllers in series: the output of one stage is the reference of the next.
///
/// A cascade is how a current loop inside a velocity loop inside a position loop
/// is written. The measurement vector carries one entry per stage, so stage `i`
/// is handed `measurement[i]`.
pub(all) struct Cascade {
  stages : Array[&@contract.Controller]
}

///|
/// A cascade of the given stages, outermost first.
pub fn Cascade::new(stages~ : Array[&@contract.Controller]) -> Cascade {
  { stages, }
}

///|
pub impl @contract.Controller for Cascade with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let mut command = reference
  for i in 0..<self.stages.length() {
    let stage_measurement = if i < measurement.len() {
      @linear.Vec::of([measurement.get(i)])
    } else {
      measurement
    }
    command = self.stages[i].update(command, stage_measurement, dt)
  }
  command
}

///|
pub impl @contract.Controller for Cascade with fn reset(self) {
  for i in 0..<self.stages.length() {
    self.stages[i].reset()
  }
}

///|
pub extend Cascade with @contract.Controller::{update, reset}

///|
/// A controller whose output is clamped into a band.
pub(all) struct Saturating {
  inner : &@contract.Controller
  output : Output
}

///|
/// A saturation wrapper.
pub fn Saturating::new(
  inner~ : &@contract.Controller,
  output~ : Output = output_unbounded,
) -> Saturating {
  { inner, output, }
}

///|
/// The band, exposed so a caller can read back the limit it set.
pub fn Saturating::output(self : Saturating) -> Output {
  self.output
}

///|
pub impl @contract.Controller for Saturating with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let raw = self.inner.update(reference, measurement, dt)
  let out = if raw.len() > 0 {
    @linear.Vec::of([clamp(raw.get(0), self.output)])
  } else {
    raw
  }
  out
}

///|
pub impl @contract.Controller for Saturating with fn reset(self) {
  self.inner.reset()
}

///|
pub extend Saturating with @contract.Controller::{update, reset}

///|
/// A controller whose output may not change faster than `rate` per second.
///
/// The limit applies to the first entry of the command, which is the one a
/// single-drive loop carries; a higher-dimensional command passes through the
/// slowest component.
pub(all) struct RateLimited {
  inner : &@contract.Controller
  rate : Double
  mut previous : Double
}

///|
/// A rate-limit wrapper. A rate that is not positive is refused.
pub fn RateLimited::new(
  inner~ : &@contract.Controller,
  rate~ : Double,
) -> RateLimited raise @linear.Refused {
  if rate <= 0.0 {
    raise @linear.Refused::Gain(got=rate)
  }
  { inner, rate, previous: 0.0, }
}

///|
pub impl @contract.Controller for RateLimited with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let raw = self.inner.update(reference, measurement, dt)
  let target = if raw.len() > 0 { raw.get(0) } else { 0.0 }
  let step = self.rate * dt
  let mut applied = target
  if target - self.previous > step {
    applied = self.previous + step
  } else if self.previous - target > step {
    applied = self.previous - step
  }
  self.previous = applied
  @linear.Vec::of([applied])
}

///|
pub impl @contract.Controller for RateLimited with fn reset(self) {
  self.inner.reset()
  self.previous = 0.0
}

///|
pub extend RateLimited with @contract.Controller::{update, reset}
