// The simulations the documentation page runs. Each function is one verb of the
// bridge; the page supplies the parameters and draws the columns.

///|
/// A PID servo on a first-order plant. The controller drives the plant model
/// `y' = (gain * u - y) / tau` and the response records the measurement and the
/// commanded output.
fn pid_trace(
  kp : Double,
  ki : Double,
  kd : Double,
  limit : Double,
  dt : Double,
  steps : Int32,
  setpoint : Double,
  gain : Double,
  tau : Double,
) -> String {
  let pid = @moonmcl.Pid::pid(kp, ki, kd, output_limit=Some(limit))
  let rows : Array[String] = []
  let mut y = 0.0
  let mut t = 0.0
  for _ in 0..<steps {
    let u = pid.update(setpoint - y, dt)
    if tau > 0.0 {
      y = y + dt * (gain * u - y) / tau
    }
    t = t + dt
    rows.push(record([t, y, u]))
  }
  table(rows)
}

///|
/// A rest-to-rest move under the jerk-limited profile, sampled evenly over the
/// whole move so the page can draw position, velocity and acceleration.
fn scurve_trace(
  distance : Double,
  v_max : Double,
  a_max : Double,
  j_max : Double,
  samples : Int32,
) -> String {
  let profile = @moonmcl.scurve_plan(distance, v_max, a_max, j_max)
  let rows : Array[String] = []
  let count = if samples < 2 { 2 } else { samples }
  let total = profile.total_time
  for i in 0..<count {
    let t = total * i.to_double() / (count - 1).to_double()
    let (s, v, a, _j) = profile.at(t)
    rows.push(record([t, s, v, a]))
  }
  table(rows)
}

///|
/// A shortest path across the grid. `blocked` holds the cells the page has
/// walled off; the answer is the sequence of cell indices from start to goal.
fn plan_trace(
  rows : Int32,
  cols : Int32,
  blocked : Array[Int32],
  start : Int32,
  goal : Int32,
  algorithm : Int32,
) -> String {
  let grid = @moonmcl.Grid::grid(rows, cols, blocked_cells=Some(blocked))
  let found = if algorithm == 1 {
    @moonmcl.dijkstra(grid, start, goal)
  } else {
    @moonmcl.astar(grid, start, goal)
  }
  match found {
    Some(path) => table(path.map(fn(cell) { cell.to_string() }))
    None => "-"
  }
}

///|
/// A Kalman filter following a constant through noisy measurements. The noise
/// is a fixed oscillation rather than a random draw so every visit sees the
/// same
/// picture, and the last column is the standard deviation of the estimate.
fn kf_trace(q : Double, r : Double, dt : Double, steps : Int32) -> String {
  let kf = @moonmcl.Kf::kf(
    @moonmcl.Vector::of([0.0]),
    @moonmcl.Matrix::of([[1.0]]),
    @moonmcl.Matrix::of([[1.0]]),
    @moonmcl.Matrix::of([[q]]),
  )
  let h = @moonmcl.Matrix::of([[1.0]])
  let noise = @moonmcl.Matrix::of([[r]])
  let truth = 2.0
  let rows : Array[String] = []
  let mut t = 0.0
  for k in 0..<steps {
    kf.predict()
    let measured = truth +
      0.9 * @math.sin(k.to_double() * 1.7) +
      0.35 * @math.cos(k.to_double() * 0.6)
    kf.update(@moonmcl.Vector::of([measured]), h, noise)
    let variance = kf.covariance().at(0, 0).unwrap_or(0.0)
    t = t + dt
    rows.push(
      record([t, truth, measured, kf.state().x(), @math.pow(variance, 0.5)]),
    )
  }
  table(rows)
}

///|
/// A differential drive: straight, then a turn, then straight again. Both the
/// inverse and the forward kinematics run every tick, so the recorded pose is
/// the one the wheels would actually produce.
fn drive_trace(
  track : Double,
  speed : Double,
  turn : Double,
  dt : Double,
  steps : Int32,
) -> String {
  let rows : Array[String] = []
  let mut pose = @moonmcl.Pose::pose(0.0, 0.0, 0.0)
  let mut t = 0.0
  for k in 0..<steps {
    let third = steps / 3
    let w = if k < third || k >= 2 * third { 0.0 } else { turn }
    let twist = @moonmcl.Twist::twist(speed, w)
    let wheels = @moonmcl.inverse_kinematics(twist, track)
    let body = @moonmcl.forward_kinematics(wheels, track)
    pose = @moonmcl.update_pose(pose, body, dt)
    t = t + dt
    rows.push(record([t, pose.x, pose.y, pose.theta, wheels.vl, wheels.vr]))
  }
  table(rows)
}

///|
/// The double integrator the modern-control demos drive, discretised at `dt`.
fn integrator(dt : Double) -> (@moonmcl.Matrix, @moonmcl.Matrix) {
  (
    @moonmcl.Matrix::of([[1.0, dt], [0.0, 1.0]]),
    @moonmcl.Matrix::of([[dt * dt / 2.0], [dt]]),
  )
}

///|
/// Receding-horizon control of the double integrator, regulated to the origin.
/// The recorded control is the first element of each plan.
fn mpc_trace(
  x0 : Double,
  v0 : Double,
  horizon : Int32,
  steps : Int32,
  q : Double,
  r : Double,
) -> String {
  let dt = 0.2
  let (a, b) = integrator(dt)
  let weight_q = @moonmcl.Matrix::of([[q, 0.0], [0.0, q]])
  let weight_r = @moonmcl.Matrix::of([[r]])
  let mpc = @moonmcl.Mpc::mpc(
    a,
    b,
    weight_q,
    weight_r,
    horizon,
    input_limit=Some(4.0),
  )
  let rows : Array[String] = []
  let mut x = @moonmcl.Vector::of([x0, v0])
  let mut t = 0.0
  for _ in 0..<steps {
    let u = mpc.step(x, @moonmcl.Vector::zeros(2))
    let force = u.x()
    let next = a.mat_vec(x).add(b.mat_vec(@moonmcl.Vector::of([force])))
    t = t + dt
    rows.push(record([t, x.x(), x.y(), force]))
    x = next
  }
  table(rows)
}

///|
/// The same double integrator under the infinite-horizon gain, for comparison
/// with the receding-horizon run.
fn lqr_trace(
  x0 : Double,
  v0 : Double,
  steps : Int32,
  q : Double,
  r : Double,
) -> String {
  let dt = 0.2
  let (a, b) = integrator(dt)
  let weight_q = @moonmcl.Matrix::of([[q, 0.0], [0.0, q]])
  let weight_r = @moonmcl.Matrix::of([[r]])
  let gain = @moonmcl.lqr_gain(a, b, weight_q, weight_r)
  let rows : Array[String] = []
  let mut x = @moonmcl.Vector::of([x0, v0])
  let mut t = 0.0
  for _ in 0..<steps {
    let force = -gain.mat_vec(x).x()
    let next = a.mat_vec(x).add(b.mat_vec(@moonmcl.Vector::of([force])))
    t = t + dt
    rows.push(record([t, x.x(), x.y(), force]))
    x = next
  }
  table(rows)
}

///|
/// Fold an angle into (-pi, pi] so a heading error never wraps past half a
/// turn.
fn wrap_angle(angle : Double) -> Double {
  @math.atan2(@math.sin(angle), @math.cos(angle))
}

///|
/// The lateral error of a point against a circle of radius `radius` centred at
/// `(0, radius)`, which is the path the two followers converge on.
fn circle_error(x : Double, y : Double, radius : Double) -> Double {
  let dx = x
  let dy = y - radius
  (@math.hypot(dx, dy) - radius).abs()
}

///|
/// Pure pursuit and Stanley on the same circular path, from the same offset
/// start. Both take the wheelbase, the speed and the lookahead distance; the
/// columns are the two trajectories and their lateral errors.
fn follow_trace(
  wheelbase : Double,
  speed : Double,
  dt : Double,
  steps : Int32,
  lookahead : Double,
) -> String {
  let radius = 8.0
  let rows : Array[String] = []
  // Pure pursuit.
  let mut pp = @moonmcl.Pose::pose(0.0, 0.0, 0.0)
  // Stanley, started 0.6 m off the path on the other side.
  let mut st = @moonmcl.Pose::pose(-0.9, -1.6, 0.0)
  let mut t = 0.0
  for _ in 0..<steps {
    // A lookahead point on the circle, one chord ahead of the robot's bearing.
    let bearing = @math.atan2(pp.x, radius - pp.y)
    let ahead = bearing + lookahead / radius
    let target_x = radius * @math.sin(ahead)
    let target_y = radius - radius * @math.cos(ahead)
    let dx = target_x - pp.x
    let dy = target_y - pp.y
    let forward = dx * @math.cos(pp.theta) + dy * @math.sin(pp.theta)
    let lateral = -dx * @math.sin(pp.theta) + dy * @math.cos(pp.theta)
    let steer = @moonmcl.pure_pursuit(
      wheelbase,
      @moonmcl.Lookahead::lookahead(lateral, forward),
      speed=Some(speed),
    )
    pp = @moonmcl.Pose::pose(
      pp.x + speed * @math.cos(pp.theta) * dt,
      pp.y + speed * @math.sin(pp.theta) * dt,
      pp.theta + speed * @math.tan(steer.angle) / wheelbase * dt,
    )
    // Stanley: the heading error against the radial tangent and the offset.
    // The path runs anticlockwise, so its inside is on the left: being outside
    // the circle is a negative left offset.
    let radius_here = @math.hypot(st.x, st.y - radius)
    let tangent = @math.atan2(st.x, radius - st.y)
    let offset = radius - radius_here
    let law = @moonmcl.stanley(
      1.4,
      wrap_angle(tangent - st.theta),
      offset,
      speed,
    )
    st = @moonmcl.Pose::pose(
      st.x + speed * @math.cos(st.theta) * dt,
      st.y + speed * @math.sin(st.theta) * dt,
      st.theta + speed * @math.tan(law.angle) / wheelbase * dt,
    )
    t = t + dt
    rows.push(
      record([
        t,
        pp.x,
        pp.y,
        circle_error(pp.x, pp.y, radius),
        st.x,
        st.y,
        circle_error(st.x, st.y, radius),
      ]),
    )
  }
  table(rows)
}
