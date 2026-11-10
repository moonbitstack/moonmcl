///|
/// The knobs of the iterative LQR.
///
/// `state_weight` and `input_weight` are the diagonal weights of the quadratic
/// cost: every state deviation costs `state_weight / 2` per unit squared and
/// every input `input_weight / 2`. `regularisation` is added to the input
/// Hessian so a direction the model cannot move stays solvable.
#derive(Debug)
pub(all) struct IlqrConfig {
  iterations : Int32
  tolerance : Double
  control : Output
  regularisation : Double
  state_weight : Double
  input_weight : Double
}

///|
/// The preset the iterative LQR runs with.
pub let ilqr_config : IlqrConfig = {
  iterations: 60,
  tolerance: 1.0e-6,
  control: output_unbounded,
  regularisation: 1.0e-6,
  state_weight: 1.0,
  input_weight: 1.0,
}

///|
/// The input sequence that drives `start` to `goal` over the horizon the initial
/// guess fixes.
///
/// The initial guess gives the horizon, the control dimension and the region the
/// solve starts in; the returned sequence has the same length. The backward pass
/// uses the model's state Jacobian and a central difference for the input one,
/// because a plant contract only promises the former.
pub fn ilqr(
  model~ : &@contract.Model,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
  controls~ : Array[@linear.Vec],
  dt~ : Double,
  config~ : IlqrConfig = ilqr_config,
) -> Array[@linear.Vec] raise @linear.Refused {
  let horizon = controls.length()
  if horizon == 0 {
    raise @linear.Refused::Empty
  }
  let n = start.len()
  if goal.len() != n {
    raise @linear.Refused::Dimension(got=goal.len(), want=n)
  }
  let m = controls[0].len()
  let state_weight = @linear.Mat::identity(n).scale(config.state_weight)
  let input_weight = @linear.Mat::identity(m).scale(config.input_weight)
  let mut u = Array::make(horizon, @linear.Vec::zeros(m))
  for k in 0..<horizon {
    u[k] = controls[k].clone()
  }
  let mut x = rollout(model, start, u, dt)
  let mut cost = total_cost(x, u, goal, state_weight, input_weight)

  for _ in 0..<config.iterations {
    let (feedforward, feedback) = backward(
      model, x, u, goal, state_weight, input_weight, dt, config,
    )
    let mut improved = false
    let mut alpha = 1.0
    for _ in 0..<12 {
      let candidate = forward_pass(
        model, x, u, feedforward, feedback, alpha, dt, config,
      )
      let candidate_cost = total_cost(
        candidate.0,
        candidate.1,
        goal,
        state_weight,
        input_weight,
      )
      if candidate_cost < cost {
        x = candidate.0
        u = candidate.1
        let delta = cost - candidate_cost
        cost = candidate_cost
        improved = true
        if delta < config.tolerance {
          return u
        }
        break
      }
      alpha = alpha * 0.5
    }
    if !improved {
      break
    }
  }
  u
}

///|
/// The states the inputs produce, with the start prepended.
fn rollout(
  model : &@contract.Model,
  start : @linear.Vec,
  controls : Array[@linear.Vec],
  dt : Double,
) -> Array[@linear.Vec] {
  let states = Array::make(controls.length() + 1, start)
  states[0] = start
  for k in 0..<controls.length() {
    states[k + 1] = model.step(states[k], controls[k], dt)
  }
  states
}

///|
/// The quadratic cost of a trajectory.
fn total_cost(
  states : Array[@linear.Vec],
  controls : Array[@linear.Vec],
  goal : @linear.Vec,
  state_weight : @linear.Mat,
  input_weight : @linear.Mat,
) -> Double {
  let mut total = 0.0
  for k in 0..<controls.length() {
    let deviation = states[k].sub(goal)
    total += 0.5 * deviation.dot(state_weight.apply(deviation))
    total += 0.5 * controls[k].dot(input_weight.apply(controls[k]))
  }
  let last = states[states.length() - 1].sub(goal)
  total + 0.5 * last.dot(state_weight.apply(last))
}

///|
/// The backward pass: a feedforward term and a feedback gain per period.
fn backward(
  model : &@contract.Model,
  states : Array[@linear.Vec],
  controls : Array[@linear.Vec],
  goal : @linear.Vec,
  state_weight : @linear.Mat,
  input_weight : @linear.Mat,
  dt : Double,
  config : IlqrConfig,
) -> (Array[@linear.Vec], Array[@linear.Mat]) raise @linear.Refused {
  let horizon = controls.length()
  let n = goal.len()
  let m = controls[0].len()
  let feedforward = Array::make(horizon, @linear.Vec::zeros(m))
  let feedback = Array::make(horizon, @linear.Mat::zeros(m, n))
  let mut vx = state_weight.apply(states[horizon].sub(goal))
  let mut vxx = state_weight
  for k in (horizon - 1)>=..0 {
    let a = model.step_jacobian(states[k], controls[k], dt)
    let b = control_jacobian(model, states[k], controls[k], dt)
    let lx = state_weight.apply(states[k].sub(goal))
    let lu = input_weight.apply(controls[k])
    let at_vx = a.transpose().apply(vx)
    let bt_vx = b.transpose().apply(vx)
    let qx = lx.add(at_vx)
    let qu = lu.add(bt_vx)
    let qxx = state_weight.add(a.transpose().mul(vxx).mul(a))
    let quu = input_weight
      .add(b.transpose().mul(vxx).mul(b))
      .add(@linear.Mat::identity(m).scale(config.regularisation))
    let qux = b.transpose().mul(vxx).mul(a)
    let step_term = @linear.solve_vec(quu, qu.scale(-1.0))
    let gain = @linear.solve(quu, qux.scale(-1.0))
    feedforward[k] = step_term
    feedback[k] = gain
    let kt = gain.transpose()
    let kt_quu = kt.mul(quu)
    vx = qx
      .add(kt_quu.apply(step_term))
      .add(kt.apply(qu))
      .add(qux.transpose().apply(step_term))
    vxx = qxx
      .add(kt_quu.mul(gain))
      .add(kt.mul(qux))
      .add(qux.transpose().mul(gain))
    ignore(n)
  }
  (feedforward, feedback)
}

///|
/// The state trajectory the updated inputs produce, with the feedforward step
/// scaled by `alpha` and the feedback taken about the previous trajectory.
fn forward_pass(
  model : &@contract.Model,
  states : Array[@linear.Vec],
  controls : Array[@linear.Vec],
  feedforward : Array[@linear.Vec],
  feedback : Array[@linear.Mat],
  alpha : Double,
  dt : Double,
  config : IlqrConfig,
) -> (Array[@linear.Vec], Array[@linear.Vec]) {
  let horizon = controls.length()
  let next_states = Array::make(horizon + 1, states[0])
  next_states[0] = states[0]
  let next_controls = Array::make(horizon, controls[0])
  for k in 0..<horizon {
    let deviation = next_states[k].sub(states[k])
    let raw = controls[k]
      .add(feedforward[k].scale(alpha))
      .add(feedback[k].apply(deviation))
    let bounded = @linear.Vec::zeros(raw.len())
    for i in 0..<raw.len() {
      bounded.set(
        i,
        clamp_scalar(raw.get(i), config.control.low, config.control.high),
      )
    }
    next_controls[k] = bounded
    next_states[k + 1] = model.step(next_states[k], bounded, dt)
  }
  (next_states, next_controls)
}

///|
/// `d step / d u` by a central difference, the derivative the contract does not
/// promise.
fn control_jacobian(
  model : &@contract.Model,
  x : @linear.Vec,
  u : @linear.Vec,
  dt : Double,
) -> @linear.Mat {
  let m = u.len()
  let rows = x.len()
  let jacobian = @linear.Mat::zeros(rows, m)
  for j in 0..<m {
    let bump = @linear.Vec::zeros(m)
    bump.set(j, @contract.jacobian_step)
    let plus = model.step(x, u.add(bump), dt)
    let minus = model.step(x, u.sub(bump), dt)
    for i in 0..<rows {
      jacobian.set(
        i,
        j,
        (plus.get(i) - minus.get(i)) / (2.0 * @contract.jacobian_step),
      )
    }
  }
  jacobian
}

///|
pub extend IlqrConfig with @debug.Debug::{to_repr}
