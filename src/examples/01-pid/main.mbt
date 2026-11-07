// A proportional-integral-differential servo driving a first-order plant to a
// setpoint, showing the output settling while the integrator removes the
// steady-state error.
//
//     moon run src/examples/01-pid

///|
#coverage.skip
fn main {
  let kp = 1.0
  let ki = 0.2
  let kd = 0.0
  let dt = 0.05
  let pid = @mctl.Pid::pid(kp, ki, kd, output_limit=Some(2.0))
  let setpoint = 1.0
  let mut value = 0.0
  for k in 0..<400 {
    let error = setpoint - value
    let u = pid.update(error, dt)
    // A first-order plant with unit time constant approaches the command.
    value = value + dt * (u - value)
    if k % 100 == 0 {
      let line = "k=" +
        k.to_string() +
        "  value=" +
        value.to_string() +
        "  output=" +
        u.to_string()
      println(line)
    }
  }
  println(
    "settled value=" +
    value.to_string() +
    " (setpoint " +
    setpoint.to_string() +
    ")",
  )
}
