///|
/// A velocity- and acceleration-feedforward term on the reference trajectory.
///
/// A feedforward does not close a loop: it removes the lag the feedback would
/// otherwise have to work off by itself. On a servo that follows a planned
/// trajectory the velocity gain dominates and the acceleration gain covers the
/// inertia.
#derive(Debug)
pub(all) struct Feedforward {
  velocity : Double
  acceleration : Double
}

///|
/// A feedforward with unit velocity gain and no acceleration gain.
pub let feedforward : Feedforward = { velocity: 1.0, acceleration: 0.0, }

///|
/// A feedforward from its two gains. A negative gain is refused.
pub fn Feedforward::new(
  velocity~ : Double = 1.0,
  acceleration~ : Double = 0.0,
) -> Feedforward raise @linear.Refused {
  if velocity < 0.0 {
    raise @linear.Refused::Gain(got=velocity)
  }
  if acceleration < 0.0 {
    raise @linear.Refused::Gain(got=acceleration)
  }
  { velocity, acceleration, }
}

///|
/// The command for a reference sample: the gains applied to its derivatives.
pub fn Feedforward::command(
  self : Feedforward,
  desired : @contract.Sample,
) -> Double {
  self.velocity * desired.velocity + self.acceleration * desired.acceleration
}

///|
/// A feedforward driven by the reference sequence: the velocity and the
/// acceleration are differenced from the references it is handed.
pub(all) struct FeedforwardRunner {
  feedforward : Feedforward
  mut previous_reference : Double
  mut previous_velocity : Double
  mut primed : Bool
}

///|
/// A runner for a feedforward.
pub fn FeedforwardRunner::new(
  feedforward~ : Feedforward = feedforward,
) -> FeedforwardRunner {
  {
    feedforward,
    previous_reference: 0.0,
    previous_velocity: 0.0,
    primed: false,
  }
}

///|
impl @contract.Controller for FeedforwardRunner with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let r = if reference.len() > 0 { reference.get(0) } else { 0.0 }
  let mut velocity = 0.0
  if self.primed && dt > 0.0 {
    velocity = (r - self.previous_reference) / dt
  }
  let acceleration = if self.primed && dt > 0.0 {
    (velocity - self.previous_velocity) / dt
  } else {
    0.0
  }
  self.previous_reference = r
  self.previous_velocity = velocity
  self.primed = true
  ignore(measurement)
  @linear.Vec::of([
    self.feedforward.velocity * velocity +
    self.feedforward.acceleration * acceleration,
  ])
}

///|
impl @contract.Controller for FeedforwardRunner with fn reset(self) {
  self.previous_reference = 0.0
  self.previous_velocity = 0.0
  self.primed = false
}

///|
pub extend FeedforwardRunner with @contract.Controller::{update, reset}

///|
pub extend Feedforward with @debug.Debug::{to_repr}
