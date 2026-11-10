// The jerk-limited profile. A jerk boundary is what separates a motion a machine
// can make from one it can only approximate: an acceleration that steps excites
// every resonance the structure has, and no amount of tuning removes the ring.
//
// The shape is seven phases: ramp the acceleration up, hold it, ramp it down,
// cruise, then mirror the pair for the deceleration. A boundary acceleration is
// taken off in its own segment before the seven and built up in one after, which
// is why the profile accepts a start and an end that are already moving and
// already accelerating.

///|
/// A jerk-limited S-curve.
pub(all) struct Scurve {
  distance : Double
  sign : Double
  duration : Double
  phases : Array[Double]
  jerks : Array[Double]
  starts : Array[(Double, Double, Double)]
}

///|
/// The sign of a number, with zero counting as positive.
fn sign_of(value : Double) -> Double {
  if value < 0.0 {
    -1.0
  } else {
    1.0
  }
}

///|
/// Advance a state by `dt` seconds at a constant `jerk`.
fn advance(
  state : (Double, Double, Double),
  jerk : Double,
  dt : Double,
) -> (Double, Double, Double) {
  let (position, velocity, acceleration) = state
  (
    position +
    velocity * dt +
    0.5 * acceleration * dt * dt +
    jerk * dt * dt * dt / 6.0,
    velocity + acceleration * dt + 0.5 * jerk * dt * dt,
    acceleration + jerk * dt,
  )
}

///|
/// The distance covered while the velocity changes by `delta`, with a
/// jerk-limited acceleration ramp on both ends and a plateau between them when
/// the limit is reached.
fn distance_of_ramp(delta : Double, a_max : Double, j_max : Double) -> Double {
  if delta <= 0.0 {
    return 0.0
  }
  let ramp_time = a_max / j_max
  let ramp_velocity = a_max * a_max / (2.0 * j_max)
  if delta >= 2.0 * ramp_velocity {
    let plateau = (delta - 2.0 * ramp_velocity) / a_max
    let raised = j_max * ramp_time * ramp_time * ramp_time / 6.0
    let held = ramp_velocity * plateau + 0.5 * a_max * plateau * plateau
    let corner = ramp_velocity + a_max * plateau
    let lowered = corner * ramp_time +
      0.5 * a_max * ramp_time * ramp_time -
      j_max * ramp_time * ramp_time * ramp_time / 6.0
    raised + held + lowered
  } else {
    let peak = (j_max * delta).sqrt()
    let half = peak / j_max
    // The two halves meet at half the velocity change; the cubic terms of the
    // two segments cancel, and what is left is the area under the velocity.
    0.5 * delta * half + 0.5 * peak * half * half
  }
}

///|
/// The time a velocity change of `delta` takes with a jerk-limited ramp on both
/// ends.
fn time_of_ramp(delta : Double, a_max : Double, j_max : Double) -> Double {
  if delta <= 0.0 {
    return 0.0
  }
  let ramp_velocity = a_max * a_max / (2.0 * j_max)
  if delta >= 2.0 * ramp_velocity {
    delta / a_max + a_max / j_max
  } else {
    2.0 * (delta / j_max).sqrt()
  }
}

///|
/// The three durations and jerks that change the velocity from `from` to `to`,
/// with `direction` telling whether this is a ramp up or the mirror of one.
fn ramp_phases(
  from : Double,
  to : Double,
  a_max : Double,
  j_max : Double,
  direction : Double,
) -> (Array[Double], Array[Double]) {
  let times = Array::make(3, 0.0)
  let jerks = Array::make(3, 0.0)
  let delta = (to - from) * direction
  if delta <= 0.0 {
    return (times, jerks)
  }
  let ramp_time = a_max / j_max
  let ramp_velocity = a_max * a_max / (2.0 * j_max)
  if delta >= 2.0 * ramp_velocity {
    times[0] = ramp_time
    times[1] = (delta - 2.0 * ramp_velocity) / a_max
    times[2] = ramp_time
  } else {
    let half = (j_max * delta).sqrt() / j_max
    times[0] = half
    times[1] = 0.0
    times[2] = half
  }
  jerks[0] = j_max * direction
  jerks[1] = 0.0
  jerks[2] = -j_max * direction
  (times, jerks)
}

///|
/// The peak velocity whose two ramps just cover `target`, and the distance they
/// cover there. `None` when even standing on the higher boundary velocity covers
/// more than the distance allows.
fn peak_for(
  target : Double,
  from : Double,
  to : Double,
  lower : Double,
  v_max : Double,
  a_max : Double,
  j_max : Double,
) -> (Double, Double)? {
  // The distance covered by a ramp is the boundary velocity times the time it
  // takes, plus the area the velocity adds above that boundary.
  let distance_at = fn(peak : Double) -> Double {
    from * time_of_ramp(peak - from, a_max, j_max) +
    distance_of_ramp(peak - from, a_max, j_max) +
    to * time_of_ramp(peak - to, a_max, j_max) +
    distance_of_ramp(peak - to, a_max, j_max)
  }
  if distance_at(lower) > target {
    return None
  }
  if distance_at(v_max) <= target {
    return Some((v_max, distance_at(v_max)))
  }
  let mut low = lower
  let mut high = v_max
  for _ in 0..<200 {
    let middle = 0.5 * (low + high)
    if distance_at(middle) <= target {
      low = middle
    } else {
      high = middle
    }
  }
  Some((low, distance_at(low)))
}

///|
/// An S-curve over `distance` with the given limits and boundary conditions.
///
/// A boundary acceleration outside the limit is refused, and so is a boundary
/// velocity against the direction of travel: neither has a jerk-limited profile
/// that starts from it.
pub fn scurve(
  distance~ : Double,
  v_max~ : Double,
  a_max~ : Double,
  j_max~ : Double,
  start_velocity~ : Double = 0.0,
  start_acceleration~ : Double = 0.0,
  end_velocity~ : Double = 0.0,
  end_acceleration~ : Double = 0.0,
) -> Scurve raise @linear.Refused {
  if v_max <= 0.0 {
    raise @linear.Refused::Gain(got=v_max)
  }
  if a_max <= 0.0 {
    raise @linear.Refused::Gain(got=a_max)
  }
  if j_max <= 0.0 {
    raise @linear.Refused::Gain(got=j_max)
  }
  if start_acceleration.abs() > a_max || end_acceleration.abs() > a_max {
    raise @linear.Refused::Unreachable
  }
  let sign = if distance < 0.0 { -1.0 } else { 1.0 }
  let span = (distance * sign).abs()
  let first_velocity = start_velocity * sign
  let last_velocity = end_velocity * sign
  let first_acceleration = start_acceleration * sign
  let last_acceleration = end_acceleration * sign
  if first_velocity < 0.0 || last_velocity < 0.0 {
    raise @linear.Refused::Unreachable
  }
  let phases : Array[Double] = []
  let jerks : Array[Double] = []
  // The starting acceleration comes off first, in a segment of its own.
  let mut position = 0.0
  let mut velocity = first_velocity
  let mut acceleration = first_acceleration
  if first_acceleration != 0.0 {
    let duration = first_acceleration.abs() / j_max
    let jerk = -sign_of(first_acceleration) * j_max
    phases.push(duration)
    jerks.push(jerk)
    let (next_position, next_velocity, _) = advance(
      (position, velocity, acceleration),
      jerk,
      duration,
    )
    position = next_position
    velocity = next_velocity
    acceleration = 0.0
  }
  // The closing acceleration is built up after the middle, so the middle has to
  // leave room for the distance that segment covers.
  let mut tail_distance = 0.0
  let mut tail_velocity = last_velocity
  let mut tail_jerk = 0.0
  let mut tail_time = 0.0
  if last_acceleration != 0.0 {
    tail_time = last_acceleration.abs() / j_max
    tail_jerk = sign_of(last_acceleration) * j_max
    tail_velocity = last_velocity - 0.5 * last_acceleration * tail_time
    tail_distance = tail_velocity * tail_time +
      tail_jerk * tail_time * tail_time * tail_time / 6.0
  }
  if tail_velocity < 0.0 {
    raise @linear.Refused::Unreachable
  }
  let target = span - position - tail_distance
  let lower = if velocity > tail_velocity { velocity } else { tail_velocity }
  let (peak, covered) = match
    peak_for(target, velocity, tail_velocity, lower, v_max, a_max, j_max) {
    None => raise @linear.Refused::Unreachable
    Some(pair) => pair
  }
  let cruise = if peak > 0.0 {
    let extra = target - covered
    if extra > 0.0 {
      extra / peak
    } else {
      0.0
    }
  } else {
    0.0
  }
  let up = ramp_phases(velocity, peak, a_max, j_max, 1.0)
  let down = ramp_phases(peak, tail_velocity, a_max, j_max, -1.0)
  for i in 0..<3 {
    phases.push(up.0[i])
    jerks.push(up.1[i])
  }
  phases.push(cruise)
  jerks.push(0.0)
  for i in 0..<3 {
    phases.push(down.0[i])
    jerks.push(down.1[i])
  }
  if last_acceleration != 0.0 {
    phases.push(tail_time)
    jerks.push(tail_jerk)
  }
  ignore(acceleration)
  let starts = Array::make(phases.length() + 1, (0.0, 0.0, 0.0))
  starts[0] = (0.0, first_velocity, first_acceleration)
  for i in 0..<phases.length() {
    starts[i + 1] = advance(starts[i], jerks[i], phases[i])
  }
  let mut duration = 0.0
  for i in 0..<phases.length() {
    duration = duration + phases[i]
  }
  { distance, sign, duration, phases, jerks, starts, }
}

///|
pub impl @contract.Trajectory for Scurve with fn duration(self) {
  self.duration
}

///|
pub impl @contract.Trajectory for Scurve with fn sample(self, t) {
  let clamped = if t < 0.0 {
    0.0
  } else if t > self.duration {
    self.duration
  } else {
    t
  }
  let mut remaining = clamped
  let mut index = 0
  while index < self.phases.length() && remaining > self.phases[index] {
    remaining = remaining - self.phases[index]
    index = index + 1
  }
  let (position, velocity, acceleration, jerk) = if index >=
    self.phases.length() {
    let last = self.starts[self.phases.length()]
    (last.0, last.1, last.2, 0.0)
  } else {
    let state = advance(self.starts[index], self.jerks[index], remaining)
    (state.0, state.1, state.2, self.jerks[index])
  }
  @contract.Sample::new(
    position=position * self.sign,
    velocity=velocity * self.sign,
    acceleration=acceleration * self.sign,
    jerk=jerk * self.sign,
  )
}

///|
pub extend Scurve with @contract.Trajectory::{duration, sample}
