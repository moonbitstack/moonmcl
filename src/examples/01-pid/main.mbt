// A PID driving a first-order plant to a setpoint, with the output limited and
// the integrator kept from winding up.
//
//     moon run src/examples/01-pid

///|
#coverage.skip
fn main {
  let pid = @mctl.Pid::new(config={
    kp: 2.0,
    ki: 1.0,
    kd: 0.1,
    weight_p: 1.0,
    weight_d: 1.0,
    derivative_on_measurement: true,
    anti_windup: @mctl.AntiWindup::Conditional,
    tracking_time: 0.05,
    filter: 1.0,
    output: { low: -1.5, high: 1.5, },
  })
  let mut value = 0.0
  let dt = 0.02
  for step in 0..<250 {
    let command = pid.command(reference=1.0, measurement=value, dt~)
    // A first-order plant with unit time constant.
    value = value + dt * (-value + command)
    if step % 50 == 0 {
      println(
        "t=" +
        (step.to_double() * dt).to_string() +
        "  y=" +
        value.to_string() +
        "  u=" +
        command.to_string(),
      )
    }
  }
}
