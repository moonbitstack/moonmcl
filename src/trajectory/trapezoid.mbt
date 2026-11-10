///|
/// A trapezoidal velocity profile: accelerate, cruise, decelerate.
///
/// It is the cheapest profile that respects a velocity and an acceleration
/// limit, and the one that puts a step in the acceleration at each corner. The
/// boundary velocities may be anything up to the peak the distance allows.
#derive(Debug)
pub(all) struct Trapezoid {
  distance : Double
  v_max : Double
  a_max : Double
  start_velocity : Double
  end_velocity : Double
  duration : Double
  ramp_time : Double
  cruise_time : Double
  peak_velocity : Double
  sign : Double
}

///|
/// A trapezoid over `distance` with the given limits and boundary velocities.
///
/// The boundary velocities are read along the direction of travel: a negative
/// distance with a start velocity of `-0.5` means the axis is already moving the
/// way it will go. A velocity against the direction of travel is refused, since
/// the profile would have to reverse inside itself.
pub fn trapezoid(
  distance~ : Double,
  v_max~ : Double,
  a_max~ : Double,
  start_velocity~ : Double = 0.0,
  end_velocity~ : Double = 0.0,
) -> Trapezoid raise @linear.Refused {
  if v_max <= 0.0 {
    raise @linear.Refused::Gain(got=v_max)
  }
  if a_max <= 0.0 {
    raise @linear.Refused::Gain(got=a_max)
  }
  let sign = if distance < 0.0 { -1.0 } else { 1.0 }
  let span = (distance * sign).abs()
  let first = start_velocity * sign
  let last = end_velocity * sign
  if first < 0.0 || last < 0.0 {
    raise @linear.Refused::Unreachable
  }
  // The peak the distance allows if there is no room to cruise.
  let peak = {
    let reachable = ((2.0 * a_max * span + first * first + last * last) / 2.0).sqrt()
    if reachable < v_max {
      reachable
    } else {
      v_max
    }
  }
  let ramp_acceleration = (peak - first) / a_max
  let ramp_deceleration = (peak - last) / a_max
  let accelerated = (peak * peak - first * first) / (2.0 * a_max)
  let decelerated = (peak * peak - last * last) / (2.0 * a_max)
  let cruise_distance = span - accelerated - decelerated
  let cruise_time = if peak > 0.0 && cruise_distance > 0.0 {
    cruise_distance / peak
  } else {
    0.0
  }
  {
    distance,
    v_max,
    a_max,
    start_velocity,
    end_velocity,
    duration: ramp_acceleration + cruise_time + ramp_deceleration,
    ramp_time: ramp_acceleration,
    cruise_time,
    peak_velocity: peak,
    sign,
  }
}

///|
/// The distance, velocity and acceleration at `t` seconds, in the direction of
/// travel.
fn Trapezoid::folded(self : Trapezoid, t : Double) -> (Double, Double, Double) {
  let accel_time = self.ramp_time
  let cruise_end = self.ramp_time + self.cruise_time
  let first = self.start_velocity * self.sign
  let last = self.end_velocity * self.sign
  let peak = self.peak_velocity
  let total = self.duration
  if t <= 0.0 {
    return (0.0, first, self.a_max)
  }
  if t >= total {
    let travelled = self.distance * self.sign
    return (travelled, last, -self.a_max)
  }
  if t < accel_time {
    // Accelerating from `first` at `a_max`.
    return (
      first * t + 0.5 * self.a_max * t * t,
      first + self.a_max * t,
      self.a_max,
    )
  }
  let accelerated = (peak * peak - first * first) / (2.0 * self.a_max)
  if t < cruise_end {
    return (accelerated + peak * (t - accel_time), peak, 0.0)
  }
  let cruise = peak * self.cruise_time
  let decelerating = t - cruise_end
  let position = accelerated +
    cruise +
    peak * decelerating -
    0.5 * self.a_max * decelerating * decelerating
  (position, peak - self.a_max * decelerating, -self.a_max)
}

///|
pub impl @contract.Trajectory for Trapezoid with fn duration(self) {
  self.duration
}

///|
pub impl @contract.Trajectory for Trapezoid with fn sample(self, t) {
  let (position, velocity, acceleration) = self.folded(t)
  @contract.Sample::new(
    position=position * self.sign,
    velocity=velocity * self.sign,
    acceleration=acceleration * self.sign,
    jerk=0.0,
  )
}

///|
pub extend Trapezoid with @contract.Trajectory::{duration, sample}

///|
pub extend Trapezoid with @debug.Debug::{to_repr}
