// The jerk-limited profile. A jerk boundary is what separates a motion a machine
// can make from one it can only approximate: an acceleration that steps excites
// every resonance the structure has, and no amount of tuning removes the ring.
//
// The profile is the shortest one that starts and ends at the given velocities
// and accelerations. Its shape is two jerk-limited ramps of the acceleration —
// the first from the starting acceleration up to a peak and back to zero, the
// second from zero down to the closing acceleration — with a cruise between them
// when the distance is long enough. A start or end acceleration is therefore part
// of the first or last ramp, not a segment of its own: taking it off first would
// spend time the machine does not have to spend.
//
// Which peak the profile runs at is the one open number. A longer distance wants
// a higher peak and lets the cruise grow; a shorter one can only afford a lower
// peak, and at some distance the cruise vanishes and the two ramps meet. That
// makes the peak a single parameter to search, and the search is what makes the
// profile time optimal rather than merely feasible.

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
/// One piece of the acceleration profile: a jerk held for a time.
///
/// A leg is the unit the solver works in; the phases of an `Scurve` are these
/// legs with the zero-length ones dropped.
type Leg = (Double, Double)

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
/// The state after a run of legs, starting from rest at the origin.
fn run(
  legs : Array[Leg],
  velocity : Double,
  acceleration : Double,
) -> (Double, Double, Double, Double) {
  let mut position = 0.0
  let mut v = velocity
  let mut a = acceleration
  let mut t = 0.0
  for leg in legs {
    let (jerk, time) = leg
    let (p, nv, na) = advance((position, v, a), jerk, time)
    position = p
    v = nv
    a = na
    t = t + time
  }
  (v, a, position, t)
}

///|
/// The fastest jerk-limited change of the acceleration from `a_in` to `a_out`
/// that adds `dv` to the velocity, or `None` when the limits do not allow it.
///
/// The acceleration runs through one extremum, `e`, which sits above both ends or
/// below both; when `e` reaches a limit the profile holds there. Whichever of the
/// two branches admits `dv` and takes less time wins.
fn transition(
  a_in : Double,
  a_out : Double,
  dv : Double,
  a_max : Double,
  j_max : Double,
) -> Array[Leg]? {
  if dv.abs() < 1.0e-12 && (a_in - a_out).abs() < 1.0e-12 {
    return Some([])
  }
  let upper_start = if a_in > a_out { a_in } else { a_out }
  let lower_start = if a_in < a_out { a_in } else { a_out }
  let sums = a_in * a_in + a_out * a_out
  let upper_area = fn(e : Double) -> Double {
    (2.0 * e * e - sums) / (2.0 * j_max)
  }
  let upper_time = fn(e : Double) -> Double {
    (e - a_in) / j_max + (e - a_out) / j_max
  }
  let lower_area = fn(e : Double) -> Double {
    (sums - 2.0 * e * e) / (2.0 * j_max)
  }
  let lower_time = fn(e : Double) -> Double {
    (a_in - e) / j_max + (a_out - e) / j_max
  }
  let times : Array[Double] = []
  let ups : Array[Double] = []
  let peaks : Array[Double] = []
  let holds : Array[Double] = []
  let top_area = upper_area(a_max)
  if dv > top_area + 1.0e-12 {
    let hold = (dv - top_area) / a_max
    times.push(upper_time(a_max) + hold)
    ups.push(1.0)
    peaks.push(a_max)
    holds.push(hold)
  } else if dv >= upper_area(upper_start) - 1.0e-12 {
    let inner = 2.0 * j_max * dv + sums
    let e = ((if inner > 0.0 { inner } else { 0.0 }) / 2.0).sqrt()
    times.push(upper_time(e))
    ups.push(1.0)
    peaks.push(e)
    holds.push(0.0)
  }
  let bottom_area = lower_area(-a_max)
  if dv < bottom_area - 1.0e-12 {
    let hold = (bottom_area - dv) / a_max
    times.push(lower_time(-a_max) + hold)
    ups.push(-1.0)
    peaks.push(-a_max)
    holds.push(hold)
  } else if dv <= lower_area(lower_start) + 1.0e-12 {
    let inner = sums - 2.0 * j_max * dv
    let e = -((if inner > 0.0 { inner } else { 0.0 }) / 2.0).sqrt()
    times.push(lower_time(e))
    ups.push(-1.0)
    peaks.push(e)
    holds.push(0.0)
  }
  if times.length() == 0 {
    return None
  }
  let mut choice = 0
  for i in 1..<times.length() {
    if times[i] < times[choice] {
      choice = i
    }
  }
  let up = ups[choice]
  let e = peaks[choice]
  let hold = holds[choice]
  if up > 0.0 {
    Some([
      (j_max, ((e - a_in) / j_max).max(0.0)),
      (0.0, hold.max(0.0)),
      (-j_max, ((e - a_out) / j_max).max(0.0)),
    ])
  } else {
    Some([
      (-j_max, ((a_in - e) / j_max).max(0.0)),
      (0.0, hold.max(0.0)),
      (j_max, ((a_out - e) / j_max).max(0.0)),
    ])
  }
}

///|
/// The profile that cruises at `peak`: ramps up to it, holds it, ramps down.
///
/// `None` when the two ramps already cover more than the distance, or when the
/// result does not land on the closing state.
fn profile_at(
  peak : Double,
  distance : Double,
  start_velocity : Double,
  start_acceleration : Double,
  end_velocity : Double,
  end_acceleration : Double,
  a_max : Double,
  j_max : Double,
) -> Array[Leg]? {
  let up = transition(
    start_acceleration,
    0.0,
    peak - start_velocity,
    a_max,
    j_max,
  )
  let down = transition(
    0.0,
    end_acceleration,
    end_velocity - peak,
    a_max,
    j_max,
  )
  match (up, down) {
    (Some(first), Some(second)) => {
      let (_, _, up_distance, _) = run(
        first, start_velocity, start_acceleration,
      )
      let (_, _, down_distance, _) = run(second, peak, 0.0)
      if up_distance + down_distance > distance + 1.0e-12 {
        return None
      }
      let cruise = if peak < 1.0e-12 {
        0.0
      } else {
        let room = (distance - up_distance - down_distance) / peak
        if room < -1.0e-9 {
          return None
        }
        room.max(0.0)
      }
      let legs : Array[Leg] = []
      for leg in first {
        legs.push(leg)
      }
      legs.push((0.0, cruise))
      for leg in second {
        legs.push(leg)
      }
      let (v, a, p, _) = run(legs, start_velocity, start_acceleration)
      // A peak the search wanders into can be outside what the limits allow; the
      // state check is what keeps such a profile out of the answer.
      if (v - end_velocity).abs() > 1.0e-7 ||
        (a - end_acceleration).abs() > 1.0e-7 ||
        (p - distance).abs() > 1.0e-7 {
        return None
      }
      Some(legs)
    }
    _ => None
  }
}

///|
/// The time of a leg list, or infinity when no profile exists.
fn profile_time(legs : Array[Leg]?) -> Double {
  match legs {
    None => 1.0e30
    Some(list) => {
      let mut total = 0.0
      for leg in list {
        total = total + leg.1
      }
      total
    }
  }
}

///|
/// The shortest profile: the cruising one when the distance allows it, otherwise
/// the peak that just covers the distance.
///
/// The peak is searched rather than solved for, because the formula for it is
/// piecewise and the piece changes with the boundary accelerations; eighty
/// halvings of a bracket take the answer to the last bits of a double.
fn solve_legs(
  distance : Double,
  start_velocity : Double,
  start_acceleration : Double,
  end_velocity : Double,
  end_acceleration : Double,
  v_max : Double,
  a_max : Double,
  j_max : Double,
) -> Array[Leg]? {
  let build = fn(peak : Double) -> Array[Leg]? {
    profile_at(
      peak, distance, start_velocity, start_acceleration, end_velocity, end_acceleration,
      a_max, j_max,
    )
  }
  let span = if v_max > start_velocity { v_max } else { start_velocity }
  let span = if end_velocity > span { end_velocity } else { span }
  // A coarse scan finds the bracket, the search below refines inside it.
  let steps = 96
  let mut best : Array[Leg]? = None
  let mut best_time = 1.0e30
  let mut best_step = 0
  for i in 0..<=steps {
    let peak = span * i.to_double() / steps.to_double()
    let legs = build(peak)
    let time = profile_time(legs)
    if time < best_time {
      best_time = time
      best = legs
      best_step = i
    }
  }
  match best {
    None => None
    Some(_) => {
      let mut low = span *
        (if best_step > 0 { best_step - 1 } else { 0 }).to_double() /
        steps.to_double()
      let mut high = span *
        (if best_step < steps { best_step + 1 } else { steps }).to_double() /
        steps.to_double()
      for _ in 0..<80 {
        let third = (high - low) / 3.0
        let left = build(low + third)
        let right = build(high - third)
        if profile_time(left) <= profile_time(right) {
          high = high - third
          if profile_time(left) < best_time {
            best_time = profile_time(left)
            best = left
          }
        } else {
          low = low + third
          if profile_time(right) < best_time {
            best_time = profile_time(right)
            best = right
          }
        }
      }
      best
    }
  }
}

///|
/// An S-curve over `distance` with the given limits and boundary conditions.
///
/// The start and end accelerations are folded into the first and last ramp, so a
/// motion that is already accelerating keeps its acceleration instead of spending
/// time on coming back to zero. A boundary acceleration outside the limit is
/// refused, and so is a boundary velocity against the direction of travel or
/// above the velocity limit: none of those has a jerk-limited profile that starts
/// from it.
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
  if first_velocity > v_max || last_velocity > v_max {
    raise @linear.Refused::Unreachable
  }
  let legs = match
    solve_legs(
      span, first_velocity, first_acceleration, last_velocity, last_acceleration,
      v_max, a_max, j_max,
    ) {
    None => raise @linear.Refused::Unreachable
    Some(found) => found
  }
  let phases : Array[Double] = []
  let jerks : Array[Double] = []
  for leg in legs {
    if leg.1 > 1.0e-12 {
      phases.push(leg.1)
      jerks.push(leg.0)
    }
  }
  if phases.length() == 0 {
    phases.push(0.0)
    jerks.push(0.0)
  }
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
