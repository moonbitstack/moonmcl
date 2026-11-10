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
  let config = @mctl.pid_gains(kp=kp.abs(), ki=ki.abs(), kd=kd.abs()) catch {
    _ => @mctl.pid_config
  }
  let pid = @mctl.Pid::new(config={
    kp: config.kp,
    ki: config.ki,
    kd: config.kd,
    weight_p: config.weight_p,
    weight_d: config.weight_d,
    derivative_on_measurement: true,
    anti_windup: config.anti_windup,
    tracking_time: config.tracking_time,
    filter: config.filter,
    output: { low: -limit, high: limit, },
  })
  let rows : Array[String] = []
  let mut y = 0.0
  let mut t = 0.0
  for _ in 0..<steps {
    let u = pid.command(reference=setpoint, measurement=y, dt~)
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
  let profile = @mctl.scurve(distance~, v_max~, a_max~, j_max~) catch {
    _ => return "ERR impossible"
  }
  let rows : Array[String] = []
  let count = if samples < 2 { 2 } else { samples }
  let total = profile.duration()
  for i in 0..<count {
    let t = total * i.to_double() / (count - 1).to_double()
    let sample = profile.sample(t)
    rows.push(
      record([t, sample.position, sample.velocity, sample.acceleration]),
    )
  }
  table(rows)
}

///|
/// A shortest path across the grid. `blocked` holds the cells the page has
/// walled off; the answer is the sequence of cell indices from start to goal.
fn plan_trace(
  lines : Int32,
  columns : Int32,
  blocked : Array[Int32],
  start : Int32,
  goal : Int32,
  algorithm : Int32,
) -> String {
  // The page numbers its cells with the column count as the width.
  let grid = @grid.Grid::new(columns, lines)
  for cell in blocked {
    grid.block(grid.x(cell), grid.y(cell))
  }
  let search = if algorithm == 1 {
    @mctl.GridSearch::Dijkstra
  } else if algorithm == 2 {
    @mctl.GridSearch::Jps
  } else if algorithm == 3 {
    @mctl.GridSearch::ThetaStar
  } else {
    @mctl.GridSearch::AStar
  }
  try {
    let path = @mctl.grid_path(grid, start~, goal~, search~)
    table(path.map(fn(cell) { cell.to_string() }))
  } catch {
    _ => "-"
  }
}

///|
/// A Kalman filter following a constant through noisy measurements. The noise
/// of the process and of the sensor are the page's two knobs, and the columns
/// are the truth, the measurement, the estimate and the standard deviation the
/// filter believes.
fn kf_trace(q : Double, r : Double, dt : Double, steps : Int32) -> String {
  let model = @mctl.StateSpace::discrete(
    a=@mctl.Mat::of([[1.0]]),
    b=@mctl.Mat::of([[0.0]]),
    c=@mctl.Mat::of([[1.0]]),
    d=@mctl.Mat::zeros(1, 1),
    dt=if dt > 0.0 { dt } else { 0.1 },
  ) catch {
    _ => return "ERR refused"
  }
  let filter = @mctl.Kalman::new(
    model~,
    process=@mctl.Mat::of([[q]]),
    measurement=@mctl.Mat::of([[r]]),
    state=@mctl.Vec::of([0.0]),
    covariance=@mctl.Mat::of([[1.0]]),
  ) catch {
    _ => return "ERR refused"
  }
  let truth = 2.0
  let rows : Array[String] = []
  let mut t = 0.0
  for k in 0..<steps {
    try {
      filter.predict(@mctl.Vec::of([0.0]), dt)
      let measured = truth +
        0.9 * @math.sin(k.to_double() * 1.7) +
        0.35 * @math.cos(k.to_double() * 0.6)
      filter.correct(@mctl.Vec::of([measured]))
      let variance = filter.covariance().get(0, 0)
      t = t + dt
      rows.push(
        record([t, truth, measured, filter.state().get(0), variance.sqrt()]),
      )
    } catch {
      _ => ()
    }
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
  let base = @mctl.Differential::new(wheel_radius=0.1, track~) catch {
    _ => return "ERR refused"
  }
  let rows : Array[String] = []
  let mut pose = @mctl.Vec::of([0.0, 0.0, 0.0])
  let mut t = 0.0
  for k in 0..<steps {
    let third = steps / 3
    let rotation = if k < third || k >= 2 * third { 0.0 } else { turn }
    let twist = @mctl.Vec::of([speed, rotation])
    let (wheels, body) = try {
      let speeds = base.inverse(twist)
      (speeds, base.forward(speeds))
    } catch {
      _ => return table(rows)
    }
    pose = base.advance(pose~, twist=body, dt~)
    t = t + dt
    rows.push(
      record([
        t,
        pose.get(0),
        pose.get(1),
        pose.get(2),
        wheels.get(0),
        wheels.get(1),
      ]),
    )
  }
  table(rows)
}

///|
/// The double integrator the modern-control demos drive, discretised at `dt`.
fn integrator(dt : Double) -> (@mctl.Mat, @mctl.Mat) {
  (
    @mctl.Mat::of([[1.0, dt], [0.0, 1.0]]),
    @mctl.Mat::of([[dt * dt / 2.0], [dt]]),
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
  let model = @mctl.StateSpace::discrete(
    a~,
    b~,
    c=@mctl.Mat::identity(2),
    d=@mctl.Mat::zeros(2, 1),
    dt~,
  ) catch {
    _ => return "ERR refused"
  }
  let controller = @mctl.Mpc::new(
    model~,
    state_weight=@mctl.Mat::of([[q, 0.0], [0.0, q]]),
    input_weight=@mctl.Mat::of([[r]]),
    config={
      horizon: if horizon < 2 {
        2
      } else if horizon > 60 {
        60
      } else {
        horizon
      },
      iterations: 400,
      penalty: 2.0,
      input: { low: -4.0, high: 4.0, },
      rate: { low: -50.0, high: 50.0, },
    },
  ) catch {
    _ => return "ERR refused"
  }
  let rows : Array[String] = []
  let mut x = @mctl.Vec::of([x0, v0])
  let mut t = 0.0
  for _ in 0..<steps {
    let command = controller.control(state=x, target=@mctl.Vec::zeros(2)) catch {
      _ => return table(rows)
    }
    let force = command.get(0)
    let next = a.apply(x).add(b.apply(@mctl.Vec::of([force])))
    t = t + dt
    rows.push(record([t, x.get(0), x.get(1), force]))
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
  let regulator = @mctl.lqr(
    a~,
    b~,
    q=@mctl.Mat::of([[q, 0.0], [0.0, q]]),
    r=@mctl.Mat::of([[r]]),
  ) catch {
    _ => return "ERR refused"
  }
  let gain = regulator.gain()
  let rows : Array[String] = []
  let mut x = @mctl.Vec::of([x0, v0])
  let mut t = 0.0
  for _ in 0..<steps {
    let force = -gain.apply(x).get(0)
    let next = a.apply(x).add(b.apply(@mctl.Vec::of([force])))
    t = t + dt
    rows.push(record([t, x.get(0), x.get(1), force]))
    x = next
  }
  table(rows)
}

///|
/// Fold an angle into (-pi, pi] so a heading error never wraps past half a turn.
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
/// The circle the followers trace, as a polyline the follower contracts walk.
fn circle_path(radius : Double) -> Array[@mctl.Vec] {
  let samples = 360
  let points = Array::make(samples + 1, @mctl.Vec::of([0.0, 0.0]))
  for i in 0..<=samples {
    let angle = i.to_double() / samples.to_double() * 6.283185307179586
    points[i] = @mctl.Vec::of([
      radius * @math.sin(angle),
      radius - radius * @math.cos(angle),
    ])
  }
  points
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
  let path = circle_path(radius)
  let pursuit = @mctl.Pursuit::{ lookahead, wheelbase, }
  let stanley = @mctl.Stanley::{
    gain: 1.4,
    wheelbase,
    soft: if speed > 0.1 {
      speed
    } else {
      0.1
    },
  }
  let rows : Array[String] = []
  let mut pp = @mctl.Vec::of([0.0, 0.0, 0.0])
  let mut st = @mctl.Vec::of([-0.9, -1.6, 0.0])
  let mut t = 0.0
  for _ in 0..<steps {
    let steer = pursuit.steering(pose=pp, path~) catch { _ => 0.0 }
    pp = @mctl.Vec::of([
      pp.get(0) + speed * @math.cos(pp.get(2)) * dt,
      pp.get(1) + speed * @math.sin(pp.get(2)) * dt,
      pp.get(2) + speed * @math.tan(steer) / wheelbase * dt,
    ])
    let law = stanley.steering(pose=st, path~) catch { _ => 0.0 }
    st = @mctl.Vec::of([
      st.get(0) + speed * @math.cos(st.get(2)) * dt,
      st.get(1) + speed * @math.sin(st.get(2)) * dt,
      st.get(2) + speed * @math.tan(wrap_angle(law)) / wheelbase * dt,
    ])
    t = t + dt
    rows.push(
      record([
        t,
        pp.get(0),
        pp.get(1),
        circle_error(pp.get(0), pp.get(1), radius),
        st.get(0),
        st.get(1),
        circle_error(st.get(0), st.get(1), radius),
      ]),
    )
  }
  table(rows)
}
