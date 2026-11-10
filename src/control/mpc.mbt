///|
/// The knobs of the model-predictive controller.
///
/// `penalty` is the ADMM step of the quadratic program, `input` the box on the
/// command and `rate` the box on its change per second. `iterations` bounds the
/// solve, so a period can never take longer than the budget says.
#derive(Debug)
pub(all) struct MpcConfig {
  horizon : Int32
  iterations : Int32
  penalty : Double
  input : Output
  rate : Output
}

///|
/// The preset the model-predictive controller runs with: a horizon of twenty
/// periods, no limit on the command and no limit on its rate.
pub let mpc_config : MpcConfig = {
  horizon: 20,
  iterations: 600,
  penalty: 2.0,
  input: output_unbounded,
  rate: output_unbounded,
}

///|
/// A receding-horizon controller for a discrete linear plant with box limits on
/// the input and on its rate of change.
///
/// The prediction is condensed into one quadratic program over the stacked
/// inputs, which is what makes the period predictable: the matrices are built
/// and factorised once per call, and the solve is a fixed number of ADMM
/// iterations rather than a search whose length depends on the active set.
pub(all) struct Mpc {
  model : @contract.StateSpace
  state_weight : @linear.Mat
  input_weight : @linear.Mat
  config : MpcConfig
  mut previous : @linear.Vec
}

///|
/// A model-predictive controller for `model` at the given weights.
///
/// The plant must be discrete: a continuous record would need the period it is
/// to be discretised at, which the controller cannot guess.
pub fn Mpc::new(
  model~ : @contract.StateSpace,
  state_weight~ : @linear.Mat,
  input_weight~ : @linear.Mat,
  config~ : MpcConfig = mpc_config,
) -> Mpc raise @linear.Refused {
  if !model.is_discrete() {
    raise @linear.Refused::Unreachable
  }
  if config.horizon < 1 {
    raise @linear.Refused::Range(
      got=config.horizon.to_double(),
      low=1.0,
      high=1.0e9,
    )
  }
  let n = model.a.rows
  if state_weight.rows != n || state_weight.cols != n {
    raise @linear.Refused::Dimension(got=state_weight.rows, want=n)
  }
  let m = model.b.cols
  if input_weight.rows != m || input_weight.cols != m {
    raise @linear.Refused::Dimension(got=input_weight.rows, want=m)
  }
  {
    model,
    state_weight,
    input_weight,
    config,
    previous: @linear.Vec::zeros(m),
  }
}

///|
/// The command for this period, after which the horizon shifts by one.
pub fn Mpc::control(
  self : Mpc,
  state~ : @linear.Vec,
  target~ : @linear.Vec,
) -> @linear.Vec raise @linear.Refused {
  let n = self.model.a.rows
  let m = self.model.b.cols
  let horizon = self.config.horizon
  if state.len() != n {
    raise @linear.Refused::Dimension(got=state.len(), want=n)
  }
  if target.len() != n {
    raise @linear.Refused::Dimension(got=target.len(), want=n)
  }
  let dt = match self.model.dt {
    Some(value) => value
    None => raise @linear.Refused::Unreachable
  }
  // The predicted states: x = sx * x0 + su * u.
  let sx = @linear.Mat::zeros(n * horizon, n)
  let su = @linear.Mat::zeros(n * horizon, m * horizon)
  let powers = Array::make(horizon + 1, @linear.Mat::identity(n))
  for i in 1..<=horizon {
    powers[i] = self.model.a.mul(powers[i - 1])
  }
  for i in 0..<horizon {
    for r in 0..<n {
      for c in 0..<n {
        sx.set(i * n + r, c, powers[i + 1].get(r, c))
      }
    }
    for j in 0..<=i {
      let block = powers[i - j].mul(self.model.b)
      for r in 0..<n {
        for c in 0..<m {
          su.set(i * n + r, j * m + c, block.get(r, c))
        }
      }
    }
  }
  let qbar = repeat_diagonal(self.state_weight, horizon)
  let rbar = repeat_diagonal(self.input_weight, horizon)
  let reference = @linear.Mat::zeros(n * horizon, 1)
  for i in 0..<horizon {
    for r in 0..<n {
      reference.set(i * n + r, 0, target.get(r))
    }
  }
  let free = sx.apply(state).sub(reference.col(0))
  let h = su.transpose().mul(qbar).mul(su).add(rbar).scale(2.0)
  let f = su.transpose().mul(qbar).apply(free).scale(2.0)
  let width = m * horizon
  let lower = @linear.Vec::zeros(width)
  let upper = @linear.Vec::zeros(width)
  for k in 0..<horizon {
    for c in 0..<m {
      lower.set(k * m + c, self.config.input.low)
      upper.set(k * m + c, self.config.input.high)
    }
  }
  let difference = difference_matrix(m, horizon)
  let dlower = @linear.Vec::zeros(width)
  let dupper = @linear.Vec::zeros(width)
  for k in 0..<horizon {
    for c in 0..<m {
      let bound = self.config.rate.high * dt
      if k == 0 {
        // The first period differs from the command actually applied last.
        dlower.set(c, -bound + self.previous.get(c))
        dupper.set(c, bound + self.previous.get(c))
      } else {
        dlower.set(k * m + c, -bound)
        dupper.set(k * m + c, bound)
      }
    }
  }
  let stacked = constrained_qp(
    h,
    f,
    lower,
    upper,
    difference,
    dlower,
    dupper,
    self.config,
  )
  let command = @linear.Vec::zeros(m)
  for c in 0..<m {
    command.set(c, stacked.get(c))
    self.previous.set(c, stacked.get(c))
  }
  command
}

///|
/// A block diagonal matrix of `count` copies of `block`.
fn repeat_diagonal(block : @linear.Mat, count : Int32) -> @linear.Mat {
  let size = block.rows
  let out = @linear.Mat::zeros(size * count, size * count)
  for i in 0..<count {
    for r in 0..<size {
      for c in 0..<size {
        out.set(i * size + r, i * size + c, block.get(r, c))
      }
    }
  }
  out
}

///|
/// The matrix that differences the stacked input: entry `k` of the product is
/// `u[k] - u[k-1]`, with `u[-1] = 0` because the caller shifts the bounds.
fn difference_matrix(width : Int32, horizon : Int32) -> @linear.Mat {
  let out = @linear.Mat::zeros(width * horizon, width * horizon)
  for k in 0..<horizon {
    for c in 0..<width {
      out.set(k * width + c, k * width + c, 1.0)
      if k > 0 {
        out.set(k * width + c, (k - 1) * width + c, -1.0)
      }
    }
  }
  out
}

///|
/// The minimiser of `0.5 z' h z + f' z` under a box on `z` and a box on the
/// differences `difference * z`.
///
/// Two constraint sets, so ADMM keeps two dual variables and one factorisation
/// of `h + p1 i + p2 difference' difference`; both projections are clamps.
fn constrained_qp(
  h : @linear.Mat,
  f : @linear.Vec,
  lower : @linear.Vec,
  upper : @linear.Vec,
  difference : @linear.Mat,
  dlower : @linear.Vec,
  dupper : @linear.Vec,
  config : MpcConfig,
) -> @linear.Vec raise @linear.Refused {
  let n = f.len()
  let penalty = if config.penalty > 0.0 { config.penalty } else { 1.0 }
  let dt = difference.transpose().mul(difference)
  let system = h
    .add(@linear.Mat::identity(n).scale(penalty))
    .add(dt.scale(penalty))
  let factor = @linear.cholesky(system)
  let z1 = @linear.Vec::zeros(n)
  let mut z2 = @linear.Vec::zeros(n)
  let dual1 = @linear.Vec::zeros(n)
  let dual2 = @linear.Vec::zeros(n)
  for i in 0..<n {
    z1.set(i, clamp_scalar(0.0, lower.get(i), upper.get(i)))
  }
  z2 = difference.apply(z1)
  let mut x = z1.clone()
  for _ in 0..<config.iterations {
    let rhs = @linear.Vec::zeros(n)
    let projected2 = difference.transpose().apply(z2.sub(dual2))
    for i in 0..<n {
      rhs.set(
        i,
        -f.get(i) +
        penalty * (z1.get(i) - dual1.get(i)) +
        penalty * projected2.get(i),
      )
    }
    x = solve_column(factor, rhs)
    let dz = difference.apply(x)
    let previous = z1.clone()
    let mut residual = 0.0
    for i in 0..<n {
      let candidate1 = clamp_scalar(
        x.get(i) + dual1.get(i),
        lower.get(i),
        upper.get(i),
      )
      let candidate2 = clamp_scalar(
        dz.get(i) + dual2.get(i),
        dlower.get(i),
        dupper.get(i),
      )
      if (x.get(i) - candidate1).abs() > residual {
        residual = (x.get(i) - candidate1).abs()
      }
      if (dz.get(i) - candidate2).abs() > residual {
        residual = (dz.get(i) - candidate2).abs()
      }
      z1.set(i, candidate1)
      z2.set(i, candidate2)
      dual1.set(i, dual1.get(i) + x.get(i) - candidate1)
      dual2.set(i, dual2.get(i) + dz.get(i) - candidate2)
    }
    if residual <= 1.0e-8 && penalty * z1.sub(previous).max_abs() <= 1.0e-8 {
      break
    }
  }
  z1
}

///|
pub extend MpcConfig with @debug.Debug::{to_repr}
