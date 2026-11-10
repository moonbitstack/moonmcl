// The planning contracts follow the three-layer split the sampling planners
// need: a space says how to draw and join states, a validity oracle says which
// states and motions are allowed, and a planner searches. A planner written
// here knows nothing about a particular state space, and a state space added
// outside the library works with every planner here.

///|
/// How many points the default `Validity::motion` checks between two states.
pub let motion_samples : Int32 = 16

///|
/// A state space a planner can search: where to draw random states, how far
/// apart two are, and how to walk between them.
///
/// A vehicle model implements this with its `(x, y, heading)` states and its own
/// steering primitives, which is what lets a sampling planner plan a Dubins
/// path without knowing what Dubins is.
pub(open) trait Space {
  ///|
  /// A state drawn from the space's own bounds, using `rng`.
  fn sample(Self, rng : &@rand.Rng) -> @linear.Vec

  ///|
  /// The distance between two states, in the space's own metric.
  fn distance(Self, a : @linear.Vec, b : @linear.Vec) -> Double

  ///|
  /// The state `t` of the way from `a` to `b`, with `t` in `[0, 1]`. A planner
  /// relies on it staying on a feasible connection, not merely on a straight
  /// line.
  fn interpolate(Self, a : @linear.Vec, b : @linear.Vec, t : Double) -> @linear.Vec
}

///|
/// Which states and motions a motion planner may use.
///
/// Collision checking lives here, so the planners stay free of geometry.
pub(open) trait Validity {
  ///|
  /// Whether the state `x` is free.
  fn valid(Self, x : @linear.Vec) -> Bool

  ///|
  /// Whether the motion from `a` to `b` stays free. The default walks the
  /// space's own interpolation in `motion_samples` steps and checks each point,
  /// which is right for a space whose interpolation cannot cut a corner.
  fn motion(Self, space : &Space, a : @linear.Vec, b : @linear.Vec) -> Bool = _
}

///|
/// The default motion check: every point of the interpolation is valid.
impl Validity with fn motion(self, space, a, b) {
  for i in 0..<=motion_samples {
    let t = i.to_double() / motion_samples.to_double()
    if !self.valid(space.interpolate(a, b, t)) {
      return false
    }
  }
  true
}

///|
/// How long a planner may search.
#derive(Debug)
pub(all) struct Budget {
  ///
  /// The number of samples or iterations to draw before giving up.
  iterations : Int32
  ///
  /// A wall-clock limit in seconds, when the caller has a clock.
  seconds : Double?
}

///|
/// The default budget: `iterations` samples and no wall-clock limit, so a run is
/// reproducible.
pub let budget : Budget = { iterations: 2000, seconds: None, }

///|
/// A planner: a start, a goal and a budget, out comes a path of states from the
/// start to the goal.
pub(open) trait Planner {
  ///|
  /// The path, including both ends, or `Refused::Exceeded` when the budget runs
  /// out and `Refused::Unreachable` when the search space is exhausted.
  fn plan(Self, start : @linear.Vec, goal : @linear.Vec, budget : Budget) -> Array[
    @linear.Vec,
  ] raise @linear.Refused
}

///|
pub extend Budget with @debug.Debug::{to_repr}
