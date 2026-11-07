// A single-input single-output PID controller with output saturation,
// conditional-integral anti-windup, a constant feedforward term, and two
// update entry points. `update` takes the error (derivative on the error);
// `update_signals` takes the setpoint and the measurement and differentiates
// the measurement instead, so a step in the setpoint does not produce a
// derivative kick. Stateful: call one of the two once per control tick.

///|
fn clamp(x : Double, lo : Double, hi : Double) -> Double {
  if x < lo {
    lo
  } else if x > hi {
    hi
  } else {
    x
  }
}

///|
/// A single-input single-output PID controller with output saturation,
/// conditional-integral anti-windup and a constant feedforward term.
#derive(@debug.Debug)
pub struct Pid {
  // Proportional gain.
  kp : Double
  // Integral gain.
  ki : Double
  // Derivative gain.
  kd : Double
  // A non-negative output magnitude; 0.0 means the output is not clamped.
  output_limit : Double
  // Constant term added to every output.
  mut feedforward : Double
  // Accumulated integral of the error.
  mut integral : Double
  // Error at the previous tick, used by `update`.
  mut prev_error : Double
  // Measurement at the previous tick, used by `update_signals`.
  mut prev_measurement : Double
  mut has_prev : Bool
}

///|
pub extend Pid with @debug.Debug::{to_repr}

///|
/// Build a controller from its three gains. `output_limit` clamps the magnitude
/// of every output; its default `0.0` leaves the output unclamped.
pub fn Pid::pid(
  kp : Double,
  ki : Double,
  kd : Double,
  output_limit~ : Double? = Some(0.0),
) -> Pid {
  {
    kp,
    ki,
    kd,
    output_limit: output_limit.unwrap_or(0.0),
    feedforward: 0.0,
    integral: 0.0,
    prev_error: 0.0,
    prev_measurement: 0.0,
    has_prev: false,
  }
}

///|
/// Set or clear the constant feedforward term added to every output.
pub fn Pid::set_feedforward(self : Pid, feedforward : Double) -> Unit {
  self.feedforward = feedforward
}

///|
/// Clear the integrator and the stored previous values so the next update is a
/// fresh start (a setpoint change or a mode switch).
pub fn Pid::reset(self : Pid) -> Unit {
  self.integral = 0.0
  self.prev_error = 0.0
  self.prev_measurement = 0.0
  self.has_prev = false
}

///|
/// The accumulated integrator value, exposed for inspection and testing.
pub fn Pid::integral_of(self : Pid) -> Double {
  self.integral
}

///|
/// Advance by one tick of period `dt` from the error `error`, with the
/// derivative taken on the error, and return the saturated output.
pub fn Pid::update(self : Pid, error : Double, dt : Double) -> Double {
  let d = if self.has_prev {
    self.kd * (error - self.prev_error) / dt
  } else {
    0.0
  }
  let out = self.advance(error, d, dt)
  self.prev_error = error
  self.has_prev = true
  out
}

///|
/// Advance by one tick of period `dt` from the setpoint and the measurement,
/// with the derivative taken on the measurement (derivative-first), and return
/// the saturated output. A step in the setpoint therefore produces no
/// derivative kick.
pub fn Pid::update_signals(
  self : Pid,
  setpoint : Double,
  measurement : Double,
  dt : Double,
) -> Double {
  let error = setpoint - measurement
  let d = if self.has_prev {
    -self.kd * (measurement - self.prev_measurement) / dt
  } else {
    0.0
  }
  let out = self.advance(error, d, dt)
  self.prev_error = error
  self.prev_measurement = measurement
  self.has_prev = true
  out
}

///|
/// Apply the integrator with conditional anti-windup, add the proportional,
/// derivative and feedforward terms, and saturate. `error` is the error and
/// `d` the already-computed derivative term.
fn Pid::advance(self : Pid, error : Double, d : Double, dt : Double) -> Double {
  let p_term = self.kp * error
  let limit = self.output_limit
  // The output this tick would produce from the current integrator, used only
  // to decide whether integrating would make the saturation worse.
  let candidate = p_term + self.ki * self.integral + d + self.feedforward
  let at_high = limit > 0.0 && candidate > limit
  let at_low = limit > 0.0 && candidate < -limit
  // Conditional integration: when saturated, integrate only if the error would
  // drive the output back out of saturation; otherwise hold the integrator.
  let toward = (at_high && error < 0.0) || (at_low && error > 0.0)
  if (!at_high && !at_low) || toward {
    self.integral += error * dt
  }
  let out = p_term + self.ki * self.integral + d + self.feedforward
  if limit > 0.0 {
    clamp(out, -limit, limit)
  } else {
    out
  }
}
