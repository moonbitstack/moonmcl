// The quadratic program the model-predictive controller leans on. The problem
// is convex with a box for a feasible set, which is the one shape where ADMM is
// both simple and robust: each iteration is one Cholesky solve and one clamp,
// and the factorisation is reused for every period of the horizon.

///|
/// The knobs of the box-constrained quadratic program.
///
/// `penalty` is the ADMM step; too small and the dual converges slowly, too
/// large and the primal oscillates. Twice the largest diagonal of `h` is the
/// usual choice for a well-scaled problem.
#derive(Debug)
pub(all) struct QpConfig {
  iterations : Int32
  penalty : Double
  tolerance : Double
}

///|
/// The preset the quadratic program runs with.
pub let qp_config : QpConfig = {
  iterations: 600,
  penalty: 2.0,
  tolerance: 1.0e-8,
}

///|
/// The minimiser of `0.5 z' h z + f' z` inside the box `[lower, upper]`.
///
/// ADMM rather than an active-set method: the horizon of a predictive
/// controller is re-solved every period with the same `h`, so a method whose
/// cost per iteration is one triangular solve and one clamp keeps the period
/// short, and there is no active set to warm start.
pub fn box_qp(
  h~ : @linear.Mat,
  f~ : @linear.Vec,
  lower~ : @linear.Vec,
  upper~ : @linear.Vec,
  config~ : QpConfig = qp_config,
) -> @linear.Vec raise @linear.Refused {
  let n = f.len()
  if h.rows != n || h.cols != n {
    raise @linear.Refused::Dimension(got=h.rows, want=n)
  }
  if lower.len() != n || upper.len() != n {
    raise @linear.Refused::Dimension(got=lower.len(), want=n)
  }
  let penalty = if config.penalty > 0.0 { config.penalty } else { 1.0 }
  // The dual variable of the box constraint.
  let z = @linear.Vec::zeros(n)
  let dual = @linear.Vec::zeros(n)
  let mut x = @linear.Vec::zeros(n)
  for i in 0..<n {
    z.set(i, clamp_scalar(0.0, lower.get(i), upper.get(i)))
  }
  let shifted = h.add(@linear.Mat::identity(n).scale(penalty))
  let factor = @linear.cholesky(shifted)
  for _ in 0..<config.iterations {
    let rhs = @linear.Vec::zeros(n)
    for i in 0..<n {
      rhs.set(i, -f.get(i) + penalty * (z.get(i) - dual.get(i)))
    }
    x = solve_column(factor, rhs)
    let previous = z.clone()
    let mut primal = 0.0
    for i in 0..<n {
      let candidate = clamp_scalar(
        x.get(i) + dual.get(i),
        lower.get(i),
        upper.get(i),
      )
      let residual = (x.get(i) - candidate).abs()
      if residual > primal {
        primal = residual
      }
      z.set(i, candidate)
      dual.set(i, dual.get(i) + x.get(i) - candidate)
    }
    // Both residuals have to be small: a point that satisfies the box but has
    // not moved the dual is not yet the minimiser.
    if primal <= config.tolerance &&
      penalty * z.sub(previous).max_abs() <= config.tolerance {
      break
    }
  }
  z
}

///|
/// Solve `factor * x = rhs` for the lower Cholesky factor of the system.
fn solve_column(
  factor : @linear.Mat,
  rhs : @linear.Vec,
) -> @linear.Vec raise @linear.Refused {
  let n = rhs.len()
  let column = @linear.Mat::zeros(n, 1)
  for i in 0..<n {
    column.set(i, 0, rhs.get(i))
  }
  @linear.cholesky_solve(factor, column).col(0)
}

///|
/// Clamp into an inclusive band.
pub fn clamp_scalar(value : Double, low : Double, high : Double) -> Double {
  if value < low {
    low
  } else if value > high {
    high
  } else {
    value
  }
}

///|
pub extend QpConfig with @debug.Debug::{to_repr}
