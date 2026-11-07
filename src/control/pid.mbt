// A single-input single-output PID controller with output saturation and
// conditional-integral anti-windup, plus a constant feedforward term. Stateful:
// call `update` once per control tick with the error and the tick period.

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
#derive(@debug.Debug)
pub struct Pid {
  kp : Double
  ki : Double
  kd : Double
  // A non-negative output magnitude; 0.0 means the output is not clamped.
  output_limit : Double
  mut feedforward : Double
  mut integral : Double
  mut prev_error : Double
  mut has_prev : Bool
}

///|
pub extend Pid with @debug.Debug::{to_repr}

///|
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
    has_prev: false,
  }
}

///|
/// Set or clear the constant feedforward term added to every output.
pub fn Pid::set_feedforward(self : Pid, feedforward : Double) -> Unit {
  self.feedforward = feedforward
}

///|
/// Clear the integrator and the stored previous error so the next update is a
/// fresh start (a setpoint change or a mode switch).
pub fn Pid::reset(self : Pid) -> Unit {
  self.integral = 0.0
  self.prev_error = 0.0
  self.has_prev = false
}

///|
/// The accumulated integrator value, exposed for inspection and testing.
pub fn Pid::integral_of(self : Pid) -> Double {
  self.integral
}

///|
/// Advance the controller by one tick of period `dt` against the error `error`
/// and return the saturated control output.
pub fn Pid::update(self : Pid, error : Double, dt : Double) -> Double {
  let d = if self.has_prev {
    self.kd * (error - self.prev_error) / dt
  } else {
    0.0
  }
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
  self.prev_error = error
  self.has_prev = true
  if limit > 0.0 {
    clamp(out, -limit, limit)
  } else {
    out
  }
}
