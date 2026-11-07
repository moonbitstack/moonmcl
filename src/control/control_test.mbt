// Tests for the control module: PID (proportional, integral, derivative,
// anti-windup, feedforward, derivative-first), the jerk-limited S-curve, LQR
// and integral LQR (exact gain and closed-loop convergence), linear MPC
// (receding-horizon convergence), and the path followers.

///|
test "pid proportional only" {
  let p = @control.Pid::pid(2.0, 0.0, 0.0)
  assert_eq(p.update(3.0, 0.1), 6.0)
}

///|
test "pid integral accumulates" {
  let p = @control.Pid::pid(0.0, 1.0, 0.0)
  // A constant error of 1.0 for three ticks of dt=0.1: the integrator reaches
  // 0.3 (each tick integrates before forming the output), so the third output
  // is ki * 0.3 = 0.3.
  ignore(p.update(1.0, 0.1))
  ignore(p.update(1.0, 0.1))
  let out = p.update(1.0, 0.1)
  assert_eq((out - 0.3).abs() < 1.0e-12, true)
  assert_eq((p.integral_of() - 0.3).abs() < 1.0e-12, true)
}

///|
test "pid derivative on error" {
  let p = @control.Pid::pid(0.0, 0.0, 1.0)
  ignore(p.update(0.0, 0.1))
  // The step from 0 to 2: d = kd * (2 - 0)/0.1 = 20.
  let out = p.update(2.0, 0.1)
  assert_eq((out - 20.0).abs() < 1.0e-9, true)
}

///|
test "pid derivative on measurement avoids the kick" {
  // Derivative-first: a setpoint step (measurement held at 0) produces no
  // derivative spike, unlike the error path above.
  let p = @control.Pid::pid(0.0, 0.0, 1.0)
  ignore(p.update_signals(0.0, 0.0, 0.1))
  // Step the setpoint to 2 with the measurement still 0: derivative is on the
  // (unchanged) measurement, so d = 0.
  let out = p.update_signals(2.0, 0.0, 0.1)
  assert_eq(out.abs() < 1.0e-9, true)
  // A real measurement change does produce a derivative term.
  let out2 = p.update_signals(2.0, 0.1, 0.1)
  assert_eq((out2 - -1.0).abs() < 1.0e-9, true)
}

///|
test "pid anti windup bounds the output" {
  let p = @control.Pid::pid(1.0, 1.0, 0.0, output_limit=Some(5.0))
  for _ in 0..<100 {
    ignore(p.update(100.0, 0.1))
  }
  // The output is clamped to the limit despite the large error.
  assert_eq(p.update(100.0, 0.1), 5.0)
}

///|
test "pid feedforward term" {
  let p = @control.Pid::pid(1.0, 0.0, 0.0)
  p.set_feedforward(2.0)
  assert_eq(p.update(1.0, 0.1), 3.0)
}

///|
test "pid reset clears state" {
  let p = @control.Pid::pid(1.0, 1.0, 1.0)
  ignore(p.update(1.0, 0.1))
  ignore(p.update(2.0, 0.1))
  p.reset()
  assert_eq(p.integral_of(), 0.0)
  // After reset the next tick is a fresh start: proportional plus the integral
  // of this tick (ki * error * dt) = 1.0 + 0.1.
  assert_eq((p.update(1.0, 0.1) - 1.1).abs() < 1.0e-12, true)
}

///|
test "feedforward static and derivative" {
  let ff = @control.Feedforward::feedforward(kf=Some(2.0), kd=Some(0.5))
  assert_eq(ff.of(3.0, 2.0), 7.0)
}

///|
test "scurve starts and ends at rest at the distance" {
  let s = @control.scurve_plan(100.0, 5.0, 2.0, 1.0)
  assert_eq(s.total_time > 0.0, true)
  let (s0, v0, a0, _) = s.at(0.0)
  assert_eq(s0, 0.0)
  assert_eq(v0, 0.0)
  assert_eq(a0, 0.0)
  let (s1, v1, a1, _) = s.at(s.total_time)
  assert_eq((s1 - 100.0).abs() < 1.0e-6, true)
  assert_eq(v1.abs() < 1.0e-6, true)
  assert_eq(a1.abs() < 1.0e-6, true)
}

///|
test "scurve respects the velocity limit" {
  let s = @control.scurve_plan(100.0, 5.0, 2.0, 1.0)
  let mut vmax = 0.0
  let mut t = 0.0
  let dt = s.total_time / 200.0
  while t <= s.total_time + 1.0e-12 {
    let (_, v, _, _) = s.at(t)
    if v.abs() > vmax {
      vmax = v.abs()
    }
    t = t + dt
  }
  assert_eq((vmax - 5.0).abs() < 1.0e-6, true)
}

///|
test "scurve seven phase total time" {
  let s = @control.scurve_plan(100.0, 5.0, 2.0, 1.0)
  // 7-phase: tj=2, ta=0.5, s_half=15, tc=(100-30)/5=14, total=4*2+2*0.5+14.
  assert_eq((s.total_time - 24.5).abs() < 1.0e-6, true)
}

///|
test "scurve reverse direction is sign flipped" {
  let s = @control.scurve_plan(-50.0, 5.0, 2.0, 1.0)
  let (s1, v1, a1, _) = s.at(s.total_time)
  assert_eq((s1 + 50.0).abs() < 1.0e-6, true)
  assert_eq(v1.abs() < 1.0e-6, true)
  assert_eq(a1.abs() < 1.0e-6, true)
}

///|
test "lqr exact gain for a single integrator" {
  // x_{k+1} = x + u, Q = R = 1. The DARE fixed point is the golden ratio
  // P = (1 + sqrt(5)) / 2 and K = P / (1 + P) = (sqrt(5) - 1) / 2.
  let a = @linear.Matrix::of([[1.0]])
  let b = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[1.0]])
  let p = @control.dare(a, b, q, r)
  assert_eq((p.at(0, 0).unwrap() - 1.6180339887).abs() < 1.0e-9, true)
  let k = @control.lqr_gain(a, b, q, r)
  assert_eq((k.at(0, 0).unwrap() - 0.6180339887).abs() < 1.0e-9, true)
}

///|
test "lqr closed loop is stable" {
  let a = @linear.Matrix::of([[1.0]])
  let b = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[1.0]])
  let k = @control.lqr_gain(a, b, q, r)
  let mut x = 100.0
  for _ in 0..<60 {
    let u = -k.at(0, 0).unwrap() * x
    x = a.mat_vec(@linear.Vector::of([x])).x() + u
  }
  assert_eq(x.abs() < 1.0e-6, true)
}

///|
test "integral lqr removes steady state error" {
  // Discrete double integrator, dt = 0.1, tracking a position reference.
  let dt = 0.1
  let a = @linear.Matrix::of([[1.0, dt], [0.0, 1.0]])
  let b = @linear.Matrix::of([[dt * dt / 2.0], [dt]])
  let c = @linear.Matrix::of([[1.0, 0.0]])
  let q = @linear.Matrix::of([[1.0, 0.0], [0.0, 1.0]])
  let q_int = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[1.0]])
  let xref = @linear.Vector::of([1.0])
  let ilqr = @control.integral_lqr(a, b, c, q, q_int, r, xref, dt)
  let ref_v = 1.0
  let mut pos = 0.0
  let mut vel = 0.0
  let mut e = 0.0
  for _ in 0..<400 {
    let x = @linear.Vector::of([pos, vel])
    let u = ilqr.step(x, @linear.Vector::of([e])).x()
    let y = c.mat_vec(x).x()
    pos = a.mat_vec(x).x() + b.mat_vec(@linear.Vector::of([u])).x()
    vel = a.mat_vec(x).y() + b.mat_vec(@linear.Vector::of([u])).y()
    e = e + dt * (ref_v - y)
  }
  assert_eq((pos - ref_v).abs() < 0.05, true)
  assert_eq(e.abs() < 0.05, true)
}

///|
test "mpc receding horizon converges to the reference" {
  // Single integrator x_{k+1} = x + u, horizon 3, Q = R = 1, xref = 0.
  let a = @linear.Matrix::of([[1.0]])
  let b = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[1.0]])
  let mpc = @control.Mpc::mpc(a, b, q, r, 3)
  let xref = @linear.Vector::of([0.0])
  let mut x = @linear.Vector::of([10.0])
  for _ in 0..<30 {
    let u = mpc.step(x, xref)
    let nx = a.mat_vec(x).x() + u.x()
    x = @linear.Vector::of([nx])
  }
  assert_eq(x.x().abs() < 1.0e-3, true)
}

///|
test "mpc respects the input limit" {
  let a = @linear.Matrix::of([[1.0]])
  let b = @linear.Matrix::of([[1.0]])
  let q = @linear.Matrix::of([[1.0]])
  let r = @linear.Matrix::of([[1.0]])
  let mpc = @control.Mpc::mpc(a, b, q, r, 3, input_limit=Some(0.5))
  let xref = @linear.Vector::of([0.0])
  let x = @linear.Vector::of([100.0])
  let u = mpc.step(x, xref)
  assert_eq(u.x().abs() <= 0.5 + 1.0e-9, true)
}

///|
test "pure pursuit wheel angle" {
  let st = @control.pure_pursuit(1.0, @control.Lookahead::lookahead(1.0, 2.0))
  // theta = atan2(1, 2); kappa = 2 sin(theta) / 2; angle = atan(kappa * wb).
  assert_eq((st.angle - 0.420534335).abs() < 1.0e-6, true)
  assert_eq(st.speed, 1.0)
}

///|
test "stanley law" {
  let st = @control.stanley(2.0, 0.3, -1.0, 2.0)
  // angle = 0.3 + atan(-kp * y / v) = 0.3 + atan(1).
  assert_eq((st.angle - (0.3 + 0.7853981634)).abs() < 1.0e-6, true)
  assert_eq(st.speed, 2.0)
}
