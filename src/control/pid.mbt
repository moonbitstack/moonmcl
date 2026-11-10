///|
/// How a PID stops its integrator while the output is saturated.
#derive(Debug)
pub(all) enum AntiWindup {
  ///
  /// The integrator keeps integrating, so a long saturation has to be unwound
  /// before the loop responds again.
  Off
  ///
  /// The integrator stops for the periods whose error would push the output
  /// further into the limit.
  Conditional
  ///
  /// The integrator is pulled back by the difference between the unsaturated
  /// and the saturated output, over `tracking_time`.
  BackCalculation
}

///|
/// An inclusive band `[low, high]` for one scalar.
#derive(Debug)
pub(all) struct Output {
  low : Double
  high : Double
}

///|
/// A band that never binds.
pub let output_unbounded : Output = { low: -1.0e15, high: 1.0e15, }

///|
/// Every number a PID uses that is not one of its three gains.
///
/// `weight_p` and `weight_d` are the setpoint weights of the standard form: the
/// proportional term acts on `weight_p * reference - measurement` and the
/// derivative on `weight_d * reference - measurement`. A weight of zero takes
/// the reference out of that term, which is what "derivative on measurement"
/// means; `derivative_on_measurement` does exactly that for the derivative
/// without the caller having to reason about the weight.
///
/// `filter` is the fraction of the raw derivative kept each period: `1` is no
/// filtering, `0.1` smooths heavily. Values outside `(0, 1]` are read as `1`.
#derive(Debug)
pub(all) struct PidConfig {
  kp : Double
  ki : Double
  kd : Double
  weight_p : Double
  weight_d : Double
  derivative_on_measurement : Bool
  anti_windup : AntiWindup
  tracking_time : Double
  filter : Double
  output : Output
}

///|
/// A PID with no gain and no limit: the preset the constructors start from.
///
/// Conditional anti-windup, no setpoint weighting, the derivative on the error
/// and no filtering are the textbook defaults; a drive with a noisy encoder
/// switches `derivative_on_measurement` on and sets `filter`.
pub let pid_config : PidConfig = {
  kp: 0.0,
  ki: 0.0,
  kd: 0.0,
  weight_p: 1.0,
  weight_d: 1.0,
  derivative_on_measurement: false,
  anti_windup: AntiWindup::Conditional,
  tracking_time: 0.05,
  filter: 1.0,
  output: output_unbounded,
}

///|
/// A PID configuration from the three gains. A negative gain is refused: a loop
/// with a negative proportional gain is unstable by construction, and refusing
/// it here names the mistake where it is made.
pub fn pid_gains(
  kp~ : Double,
  ki~ : Double = 0.0,
  kd~ : Double = 0.0,
) -> PidConfig raise @linear.Refused {
  if kp < 0.0 {
    raise @linear.Refused::Gain(got=kp)
  }
  if ki < 0.0 {
    raise @linear.Refused::Gain(got=ki)
  }
  if kd < 0.0 {
    raise @linear.Refused::Gain(got=kd)
  }
  {
    kp,
    ki,
    kd,
    weight_p: pid_config.weight_p,
    weight_d: pid_config.weight_d,
    derivative_on_measurement: pid_config.derivative_on_measurement,
    anti_windup: pid_config.anti_windup,
    tracking_time: pid_config.tracking_time,
    filter: pid_config.filter,
    output: pid_config.output,
  }
}

///|
/// A proportional-integral-derivative controller.
///
/// The integrator, the previous derivative input and the filtered derivative are
/// the only state; `reset` clears them so a loop can start from rest.
pub(all) struct Pid {
  config : PidConfig
  mut integral : Double
  mut previous_input : Double
  mut derivative : Double
  mut primed : Bool
}

///|
/// A PID from a configuration.
pub fn Pid::new(config~ : PidConfig = pid_config) -> Pid {
  {
    config,
    integral: 0.0,
    previous_input: 0.0,
    derivative: 0.0,
    primed: false,
  }
}

///|
/// The integrator value, which is what an anti-windup scheme acts on and what a
/// test asserts when the loop sits in saturation.
pub fn Pid::integral(self : Pid) -> Double {
  self.integral
}

///|
/// The output for one period of a scalar loop.
pub fn Pid::command(
  self : Pid,
  reference~ : Double,
  measurement~ : Double,
  dt~ : Double,
) -> Double {
  let config = self.config
  let error = reference - measurement
  let derivative_input = if config.derivative_on_measurement {
    -measurement
  } else {
    config.weight_d * reference - measurement
  }
  let mut raw_derivative = 0.0
  if self.primed && dt > 0.0 {
    raw_derivative = config.kd * (derivative_input - self.previous_input) / dt
  }
  let smoothing = if config.filter > 0.0 && config.filter <= 1.0 {
    config.filter
  } else {
    1.0
  }
  let derivative = self.derivative +
    smoothing * (raw_derivative - self.derivative)
  self.previous_input = derivative_input
  self.derivative = derivative
  self.primed = true
  let proportional = config.kp * (config.weight_p * reference - measurement)
  let candidate = proportional + self.integral + derivative
  let saturated = clamp(candidate, config.output)
  match config.anti_windup {
    Off => self.integral = self.integral + config.ki * error * dt
    Conditional =>
      if candidate == saturated {
        self.integral = self.integral + config.ki * error * dt
      }
    BackCalculation =>
      if config.tracking_time > 0.0 {
        self.integral = self.integral +
          config.ki * error * dt +
          (saturated - candidate) * dt / config.tracking_time
      } else {
        self.integral = self.integral + config.ki * error * dt
      }
  }
  clamp(proportional + self.integral + derivative, config.output)
}

///|
/// Clamp `value` into the band.
fn clamp(value : Double, band : Output) -> Double {
  if value < band.low {
    band.low
  } else if value > band.high {
    band.high
  } else {
    value
  }
}

///|
pub impl @contract.Controller for Pid with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let r = if reference.len() > 0 { reference.get(0) } else { 0.0 }
  let y = if measurement.len() > 0 { measurement.get(0) } else { 0.0 }
  @linear.Vec::of([self.command(reference=r, measurement=y, dt~)])
}

///|
pub impl @contract.Controller for Pid with fn reset(self) {
  self.integral = 0.0
  self.derivative = 0.0
  self.previous_input = 0.0
  self.primed = false
}

///|
pub extend Pid with @contract.Controller::{update, reset}

///|
pub extend PidConfig with @debug.Debug::{to_repr}

///|
pub extend AntiWindup with @debug.Debug::{to_repr}

///|
pub extend Output with @debug.Debug::{to_repr}
