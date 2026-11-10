// Tests for the control package. Behaviour first: a loop that should converge
// converges, a limit that should bind binds, and a solver's answer satisfies the
// equation it is supposed to solve.

///|
fn refused(f : () -> Unit raise @linear.Refused) -> @linear.Refused? {
  try {
    f()
    None
  } catch {
    e => Some(e)
  }
}

///|
fn close(a : Double, b : Double, tolerance~ : Double = 1.0e-9) -> Bool {
  (a - b).abs() <= tolerance
}

///|
/// A first-order plant step: `y' = -y + u`.
fn plant(value : Double, command : Double, dt : Double) -> Double {
  value + dt * (-value + command)
}

// --- PID ------------------------------------------------------------------

///|
test "pid proportional term" {
  let pid = @control.Pid::new(config=@control.pid_gains(kp=2.0))
  assert_eq(
    close(pid.command(reference=3.0, measurement=0.0, dt=0.1), 6.0),
    true,
  )
}

///|
test "pid integral removes a standing error" {
  let pid = @control.Pid::new(config=@control.pid_gains(kp=1.0, ki=2.0))
  let mut value = 0.0
  for _ in 0..<4000 {
    let command = pid.command(reference=1.0, measurement=value, dt=0.01)
    value = plant(value, command, 0.01)
  }
  assert_eq(close(value, 1.0, tolerance=1.0e-4), true)
}

///|
test "pid derivative on the error" {
  let pid = @control.Pid::new(config=@control.pid_gains(kp=0.0, kd=1.0))
  ignore(pid.command(reference=0.0, measurement=0.0, dt=0.1))
  // The error steps from 0 to 2 over one period: d = 2 / 0.1.
  assert_eq(
    close(pid.command(reference=2.0, measurement=0.0, dt=0.1), 20.0),
    true,
  )
}

///|
test "pid derivative on the measurement has no setpoint kick" {
  let config = @control.pid_gains(kp=0.0, kd=1.0)
  let pid = @control.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: config.weight_p,
    weight_d: config.weight_d,
    derivative_on_measurement: true,
    anti_windup: config.anti_windup,
    tracking_time: config.tracking_time,
    filter: config.filter,
    output: config.output,
  })
  ignore(pid.command(reference=0.0, measurement=0.0, dt=0.1))
  // The setpoint jumps but the measurement does not move: no derivative term.
  assert_eq(
    close(pid.command(reference=2.0, measurement=0.0, dt=0.1), 0.0),
    true,
  )
}

///|
test "pid setpoint weight softens the proportional term" {
  let config = @control.pid_gains(kp=10.0)
  let pid = @control.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: 0.2,
    weight_d: config.weight_d,
    derivative_on_measurement: config.derivative_on_measurement,
    anti_windup: config.anti_windup,
    tracking_time: config.tracking_time,
    filter: config.filter,
    output: config.output,
  })
  // 10 * (0.2 * 1 - 0) = 2 rather than 10.
  assert_eq(
    close(pid.command(reference=1.0, measurement=0.0, dt=0.1), 2.0),
    true,
  )
}

///|
test "pid conditional anti-windup stops the integrator in saturation" {
  let config = @control.pid_gains(kp=0.0, ki=1.0)
  let pid = @control.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: config.weight_p,
    weight_d: config.weight_d,
    derivative_on_measurement: config.derivative_on_measurement,
    anti_windup: @control.AntiWindup::Conditional,
    tracking_time: config.tracking_time,
    filter: config.filter,
    output: { low: -0.5, high: 0.5, },
  })
  for _ in 0..<100 {
    ignore(pid.command(reference=1.0, measurement=0.0, dt=0.1))
  }
  // The integrator integrates until the next step would leave the band, so it
  // comes to rest one period past the limit.
  assert_eq(close(pid.integral(), 0.6), true)
}

///|
test "pid without anti-windup winds up" {
  let config = @control.pid_gains(kp=0.0, ki=1.0)
  let pid = @control.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: config.weight_p,
    weight_d: config.weight_d,
    derivative_on_measurement: config.derivative_on_measurement,
    anti_windup: @control.AntiWindup::Off,
    tracking_time: config.tracking_time,
    filter: config.filter,
    output: { low: -0.5, high: 0.5, },
  })
  for _ in 0..<100 {
    ignore(pid.command(reference=1.0, measurement=0.0, dt=0.1))
  }
  assert_eq(close(pid.integral(), 10.0, tolerance=1.0e-9), true)
}

///|
test "pid back-calculation pulls the integrator back" {
  let config = @control.pid_gains(kp=0.0, ki=10.0)
  let pid = @control.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: config.weight_p,
    weight_d: config.weight_d,
    derivative_on_measurement: config.derivative_on_measurement,
    anti_windup: @control.AntiWindup::BackCalculation,
    tracking_time: 0.1,
    filter: config.filter,
    output: { low: -0.5, high: 0.5, },
  })
  for _ in 0..<200 {
    ignore(pid.command(reference=1.0, measurement=0.0, dt=0.1))
  }
  // The correction balances this period's integration when the unsaturated
  // output sits at `ki * error * dt * tracking_time` above the limit, so the
  // integrator settles at 1.5 instead of running away.
  assert_eq(close(pid.integral(), 1.5, tolerance=1.0e-6), true)
  assert_eq(
    close(pid.command(reference=1.0, measurement=0.0, dt=0.1), 0.5),
    true,
  )
}

///|
test "pid refuses a negative gain" {
  match
    refused(fn() raise @linear.Refused { ignore(@control.pid_gains(kp=-1.0)) }) {
    Some(@linear.Refused::Gain(got~)) => assert_eq(got, -1.0)
    _ => abort("expected Gain")
  }
}

///|
test "pid fits the controller contract" {
  let pid = @control.Pid::new(config=@control.pid_gains(kp=2.0))
  let command = pid.update(@linear.Vec::of([1.0]), @linear.Vec::of([0.0]), 0.1)
  assert_eq(close(command.get(0), 2.0), true)
  pid.reset()
  assert_eq(pid.integral(), 0.0)
}

// --- feedforward and combinators ------------------------------------------

///|
test "feedforward on a reference sample" {
  let ff = @control.Feedforward::new(velocity=2.0, acceleration=0.5)
  let sample = @contract.Sample::new(
    position=1.0,
    velocity=3.0,
    acceleration=4.0,
  )
  assert_eq(close(ff.command(sample), 2.0 * 3.0 + 0.5 * 4.0), true)
}

///|
test "feedforward differentiates the reference sequence" {
  let runner = @control.FeedforwardRunner::new(feedforward={
    velocity: 1.0,
    acceleration: 0.0,
  })
  ignore(runner.update(@linear.Vec::of([0.0]), @linear.Vec::of([0.0]), 0.1))
  let command = runner.update(
    @linear.Vec::of([1.0]),
    @linear.Vec::of([0.0]),
    0.1,
  )
  assert_eq(close(command.get(0), 10.0), true)
}

///|
test "saturating clamps the inner command" {
  let inner = @control.Pid::new(config=@control.pid_gains(kp=10.0))
  let limited = @control.Saturating::new(inner~, output={
    low: -1.0,
    high: 1.0,
  })
  let command = limited.update(
    @linear.Vec::of([1.0]),
    @linear.Vec::of([0.0]),
    0.1,
  )
  assert_eq(close(command.get(0), 1.0), true)
}

///|
test "rate limiter bounds one period of change" {
  let inner = @control.Pid::new(config=@control.pid_gains(kp=10.0))
  let limited = @control.RateLimited::new(inner~, rate=5.0)
  let first = limited.update(
    @linear.Vec::of([1.0]),
    @linear.Vec::of([0.0]),
    0.1,
  )
  // The command wants 10 but 5 per second over 0.1 s allows 0.5.
  assert_eq(close(first.get(0), 0.5), true)
  let second = limited.update(
    @linear.Vec::of([1.0]),
    @linear.Vec::of([0.0]),
    0.1,
  )
  assert_eq(close(second.get(0), 1.0), true)
}

///|
test "rate limiter refuses a rate that is not positive" {
  let inner = @control.Pid::new()
  match
    refused(fn() raise @linear.Refused {
      ignore(@control.RateLimited::new(inner~, rate=0.0))
    }) {
    Some(@linear.Refused::Gain(_)) => ()
    _ => abort("expected Gain")
  }
}

///|
test "cascade feeds one stage into the next" {
  let outer = @control.Pid::new(config=@control.pid_gains(kp=1.0))
  let inner = @control.Pid::new(config=@control.pid_gains(kp=2.0))
  let cascade = @control.Cascade::new(stages=[outer, inner])
  // The outer stage sees an error of 1 and asks for 1; the inner stage turns
  // that into 2 with its own measurement of zero.
  let command = cascade.update(
    @linear.Vec::of([1.0]),
    @linear.Vec::of([0.0, 0.0]),
    0.1,
  )
  assert_eq(close(command.get(0), 2.0), true)
}

// --- Riccati, LQR ---------------------------------------------------------

///|
/// A double integrator sampled at `dt`.
fn double_integrator(dt : Double) -> (@linear.Mat, @linear.Mat) {
  (
    @linear.Mat::of([[1.0, dt], [0.0, 1.0]]),
    @linear.Mat::of([[0.5 * dt * dt], [dt]]),
  )
}

///|
test "dare satisfies its own equation" {
  let (a, b) = double_integrator(0.1)
  let q = @linear.Mat::identity(2)
  let r = @linear.Mat::of([[1.0]])
  let solution = @control.dare(a~, b~, q~, r~, tolerance=1.0e-14)
  let p = solution.cost
  let btp = b.transpose().mul(p)
  let s = r.add(btp.mul(b))
  let bpa = btp.mul(a)
  let k = @linear.solve(s, bpa)
  let residual = q.add(a.transpose().mul(p).mul(a)).sub(bpa.transpose().mul(k))
  assert_eq(residual.approx(p, tolerance=1.0e-9), true)
  // The gain the recursion ends on is the one the cost implies.
  assert_eq(k.approx(solution.gain, tolerance=1.0e-9), true)
}

///|
test "lqr drives a double integrator to its reference" {
  let (a, b) = double_integrator(0.1)
  let regulator = @control.lqr(
    a~,
    b~,
    q=@linear.Mat::diagonal(@linear.Vec::of([10.0, 1.0])),
    r=@linear.Mat::of([[1.0]]),
  )
  let mut state = @linear.Vec::of([1.0, 0.0])
  let reference = @linear.Vec::of([0.0, 0.0])
  for _ in 0..<200 {
    let command = regulator.update(reference, state, 0.1)
    state = a.apply(state).add(b.apply(command))
  }
  assert_eq(state.max_abs() < 1.0e-6, true)
}

///|
test "integral lqr removes a constant disturbance" {
  let (a, b) = double_integrator(0.1)
  let regulator = @control.integral_lqr(
    a~,
    b~,
    q=@linear.Mat::identity(2),
    r=@linear.Mat::of([[1.0]]),
    c=@linear.Mat::of([[1.0, 0.0]]),
    integral_weight=@linear.Mat::of([[5.0]]),
    dt=0.1,
  )
  let mut state = @linear.Vec::of([0.0, 0.0])
  let target = @linear.Vec::of([1.0, 0.0])
  for _ in 0..<600 {
    let command = regulator.update(target, state, 0.1)
    // A constant acceleration disturbance of 0.02 per period.
    state = a
      .apply(state)
      .add(b.apply(command))
      .add(@linear.Vec::of([0.0, 0.02]))
  }
  // The integrator takes the position offset to zero; the velocity keeps the
  // offset the disturbance puts on it, which is what the plant allows.
  assert_eq(close(state.get(0), 1.0, tolerance=2.0e-3), true)
  assert_eq(state.get(1).abs() < 5.0e-2, true)
}

// --- quadratic program and MPC --------------------------------------------

///|
test "box qp finds the unconstrained optimum inside the box" {
  let h = @linear.Mat::of([[2.0, 0.0], [0.0, 4.0]])
  let f = @linear.Vec::of([-2.0, -8.0])
  let solution = @control.box_qp(
    h~,
    f~,
    lower=@linear.Vec::of([-10.0, -10.0]),
    upper=@linear.Vec::of([10.0, 10.0]),
  )
  // The unconstrained minimum is h^-1 * -f = [1, 2].
  assert_eq(
    solution.approx(@linear.Vec::of([1.0, 2.0]), tolerance=1.0e-5),
    true,
  )
}

///|
test "box qp respects the box" {
  let h = @linear.Mat::identity(2)
  let f = @linear.Vec::of([-5.0, 5.0])
  let solution = @control.box_qp(
    h~,
    f~,
    lower=@linear.Vec::of([-1.0, -1.0]),
    upper=@linear.Vec::of([1.0, 1.0]),
  )
  assert_eq(
    solution.approx(@linear.Vec::of([1.0, -1.0]), tolerance=1.0e-5),
    true,
  )
}

///|
test "mpc holds the input limit and converges" {
  let (a, b) = double_integrator(0.1)
  let model = @contract.StateSpace::discrete(
    a~,
    b~,
    c=@linear.Mat::identity(2),
    d=@linear.Mat::zeros(2, 1),
    dt=0.1,
  )
  let controller = @control.Mpc::new(
    model~,
    state_weight=@linear.Mat::diagonal(@linear.Vec::of([10.0, 1.0])),
    input_weight=@linear.Mat::of([[1.0]]),
    config={
      horizon: 10,
      iterations: 400,
      penalty: 2.0,
      input: { low: -0.5, high: 0.5, },
      rate: { low: -100.0, high: 100.0, },
    },
  )
  let mut state = @linear.Vec::of([0.0, 0.0])
  let target = @linear.Vec::of([1.0, 0.0])
  let mut largest = 0.0
  for _ in 0..<150 {
    let command = controller.control(state~, target~)
    let applied = if command.get(0) < -0.5 {
      -0.5
    } else if command.get(0) > 0.5 {
      0.5
    } else {
      command.get(0)
    }
    assert_eq(command.get(0) <= 0.5 + 1.0e-6, true)
    assert_eq(command.get(0) >= -0.5 - 1.0e-6, true)
    if applied.abs() > largest {
      largest = applied.abs()
    }
    state = a.apply(state).add(b.apply(@linear.Vec::of([applied])))
  }
  assert_eq(close(state.get(0), 1.0, tolerance=5.0e-2), true)
  assert_eq(state.get(1).abs() < 5.0e-2, true)
}

///|
test "mpc refuses a continuous plant" {
  let model = @contract.StateSpace::continuous(
    a=@linear.Mat::identity(2),
    b=@linear.Mat::of([[0.0], [1.0]]),
    c=@linear.Mat::identity(2),
    d=@linear.Mat::zeros(2, 1),
  )
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @control.Mpc::new(
          model~,
          state_weight=@linear.Mat::identity(2),
          input_weight=@linear.Mat::of([[1.0]]),
        ),
      )
    }) {
    Some(@linear.Refused::Unreachable) => ()
    _ => abort("expected Unreachable")
  }
}

// --- iterative LQR --------------------------------------------------------

///|
test "ilqr drives a double integrator to the goal" {
  let (a, b) = double_integrator(0.1)
  let model = @contract.StateSpace::discrete(
    a~,
    b~,
    c=@linear.Mat::identity(2),
    d=@linear.Mat::zeros(2, 1),
    dt=0.1,
  )
  let guess = Array::make(30, @linear.Vec::of([0.0]))
  let commands = @control.ilqr(
    model~,
    start=@linear.Vec::of([0.0, 0.0]),
    goal=@linear.Vec::of([1.0, 0.0]),
    controls=guess,
    dt=0.1,
  )
  // The optimum of the same problem solved as one quadratic program over the
  // stacked inputs: the iterative solve has to land on it.
  assert_eq(
    close(commands[0].get(0), 0.8888486371132297, tolerance=1.0e-6),
    true,
  )
  assert_eq(
    close(commands[5].get(0), 0.2919889673982484, tolerance=1.0e-6),
    true,
  )
  assert_eq(
    close(commands[29].get(0), -0.019835867220119454, tolerance=1.0e-6),
    true,
  )
  let mut state = @linear.Vec::of([0.0, 0.0])
  for k in 0..<commands.length() {
    state = model.step(state, commands[k], 0.1)
  }
  assert_eq(close(state.get(0), 0.85981908, tolerance=1.0e-6), true)
  assert_eq(close(state.get(1), 0.20536772, tolerance=1.0e-6), true)
}

// --- path following -------------------------------------------------------

///|
/// A straight path along the x axis.
fn straight_path() -> Array[@linear.Vec] {
  [@linear.Vec::of([0.0, 0.0]), @linear.Vec::of([10.0, 0.0])]
}

///|
test "pure pursuit asks for no steering on a straight path" {
  let pose = @linear.Vec::of([0.0, 0.0, 0.0])
  let steering = @control.pursuit.steering(pose~, path=straight_path())
  assert_eq(close(steering, 0.0, tolerance=1.0e-9), true)
}

///|
test "pure pursuit steers towards a lookahead point off the axis" {
  // The vehicle sits on the path but points 90 degrees away; the lookahead point
  // is ahead on the x axis, so the steering must be large.
  let pose = @linear.Vec::of([0.0, 0.0, 1.5707963267948966])
  let steering = @control.pursuit.steering(pose~, path=straight_path())
  assert_eq(steering < -1.0, true)
}

///|
test "stanley corrects a cross-track error" {
  let pose = @linear.Vec::of([1.0, 1.0, 0.0])
  let steering = @control.stanley.steering(pose~, path=straight_path())
  // The vehicle is to the left of the path, so the correction turns it right.
  assert_eq(steering < 0.0, true)
  let on_path = @control.stanley.steering(
    pose=@linear.Vec::of([1.0, 0.0, 0.0]),
    path=straight_path(),
  )
  assert_eq(close(on_path, 0.0, tolerance=1.0e-9), true)
}

// --- model-based effort ---------------------------------------------------

///|
test "computed torque cancels gravity and closes the loop" {
  let law = @control.ComputedTorque::new(
    inertia=fn(q) { @linear.Mat::identity(q.len()) },
    gravity=fn(q) { @linear.Vec::filled(q.len(), 9.81) },
    kp=4.0,
    kd=1.0,
  )
  let effort = law.effort(
    q=@linear.Vec::of([0.0]),
    qd=@linear.Vec::of([0.0]),
    q_desired=@linear.Vec::of([1.0]),
    qd_desired=@linear.Vec::of([0.0]),
    qdd_desired=@linear.Vec::of([0.0]),
  )
  assert_eq(close(effort.get(0), 4.0 + 9.81), true)
}

///|
test "impedance returns to the target at rest" {
  let controller = @control.Impedance::new(
    mass=1.0,
    damping=4.0,
    stiffness=4.0,
    start=@linear.Vec::of([0.0]),
  )
  let target = @linear.Vec::of([1.0])
  let zero = @linear.Vec::of([0.0])
  for _ in 0..<400 {
    ignore(controller.effort(target~, external=zero, dt=0.01))
  }
  assert_eq(close(controller.position().get(0), 1.0, tolerance=1.0e-2), true)
}

///|
test "impedance refuses a mass that is not positive" {
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @control.Impedance::new(
          mass=0.0,
          damping=1.0,
          stiffness=1.0,
          start=@linear.Vec::of([0.0]),
        ),
      )
    }) {
    Some(@linear.Refused::Gain(_)) => ()
    _ => abort("expected Gain")
  }
}

// --- barrier filter -------------------------------------------------------

///|
test "cbf leaves a safe command alone" {
  let safe = @control.cbf_filter(
    nominal=@linear.Vec::of([0.2]),
    gradient=@linear.Vec::of([1.0]),
    drift=0.0,
    barrier=1.0,
    alpha=1.0,
    lower=@linear.Vec::of([-1.0]),
    upper=@linear.Vec::of([1.0]),
  )
  assert_eq(safe.approx(@linear.Vec::of([0.2]), tolerance=1.0e-9), true)
}

///|
test "cbf moves an unsafe command to the boundary" {
  // h = 1, dh/dt = u, so the constraint is u + 1 >= 0.
  let safe = @control.cbf_filter(
    nominal=@linear.Vec::of([-3.0]),
    gradient=@linear.Vec::of([1.0]),
    drift=0.0,
    barrier=1.0,
    alpha=1.0,
    lower=@linear.Vec::of([-10.0]),
    upper=@linear.Vec::of([10.0]),
  )
  assert_eq(close(safe.get(0), -1.0, tolerance=1.0e-6), true)
}

///|
test "cbf respects the box when the box and the barrier conflict" {
  let safe = @control.cbf_filter(
    nominal=@linear.Vec::of([0.0]),
    gradient=@linear.Vec::of([1.0]),
    drift=0.0,
    barrier=5.0,
    alpha=1.0,
    lower=@linear.Vec::of([-1.0]),
    upper=@linear.Vec::of([1.0]),
  )
  assert_eq(safe.get(0) <= 1.0 + 1.0e-9, true)
  assert_eq(safe.get(0) >= -1.0 - 1.0e-9, true)
}
