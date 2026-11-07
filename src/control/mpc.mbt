// Linear model predictive control for the discrete system
//   x_{k+1} = A x_k + B u_k
// over a horizon of N steps, minimising
//   sum_{k=0}^{N-1} (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k
// The cost is a convex quadratic in the input sequence U, so the
// unconstrained optimum is the solution of one linear system; optional
// input saturation is then enforced with a few projected gradient steps.

///|
/// A finite-horizon linear MPC. The whole horizon is solved as one quadratic
/// program whose prediction matrices are precomputed at construction, so each
/// `step` costs a pair of matrix-vector products and one small solve.
#derive(@debug.Debug)
pub struct Mpc {
  // State transition.
  a : @linear.Matrix
  // Input matrix.
  b : @linear.Matrix
  // State weight.
  q : @linear.Matrix
  // Input weight.
  r : @linear.Matrix
  // Control horizon (number of optimised inputs).
  horizon : Int32
  // Non-negative input magnitude; 0.0 means no input limit.
  input_limit : Double
  // Projected gradient steps applied when the input is saturated.
  refine_steps : Int32
  // F, size (n*h) x n: row block i is A^i, so F x0 is the open-loop trajectory.
  f : @linear.Matrix
  // G, size (n*h) x (m*h): block (i, j) is A^(i-j-1) B for i > j.
  g : @linear.Matrix
  // Working storage for the state weighting inside the horizon.
  w : @linear.Matrix
  // Block-diagonal input weight over the horizon.
  r_tot : @linear.Matrix
  // Hessian of the cost in the input sequence.
  hess : @linear.Matrix
}

///|
pub extend Mpc with @debug.Debug::{to_repr}

///|
/// Build the controller for the discrete pair `(a, b)` with weights `q` (state)
/// and `r` (input) over `horizon` steps, and precompute the prediction matrices
/// `f` and `g` plus the cost Hessian. `input_limit` clamps the magnitude of
/// every input (the default `0.0` clamps nothing), and `refine_steps` sets the
/// projected-gradient passes spent on that clamp.
pub fn Mpc::mpc(
  a : @linear.Matrix,
  b : @linear.Matrix,
  q : @linear.Matrix,
  r : @linear.Matrix,
  horizon : Int32,
  input_limit~ : Double? = Some(0.0),
  refine_steps~ : Int32? = Some(64),
) -> Mpc {
  let n = a.rows
  let m = b.cols
  let h = horizon
  let f = @linear.Matrix::zeros(n * h, n)
  let g = @linear.Matrix::zeros(n * h, m * h)
  // Precompute A^k and A^k B for k = 0..=h.
  let mut a_pow = @linear.Matrix::identity(n)
  let mut ab_pow = b
  let powers_a = [a_pow]
  let powers_ab = [ab_pow]
  for _ in 0..<h {
    a_pow = a_pow.mat_mat(a)
    ab_pow = a.mat_mat(ab_pow)
    powers_a.push(a_pow)
    powers_ab.push(ab_pow)
  }
  for i in 0..<h {
    // Row block i of F is A^i.
    for ii in 0..<n {
      for jj in 0..<n {
        let v = powers_a[i].at(ii, jj).unwrap_or(0.0)
        if v != 0.0 {
          f.set(i * n + ii, jj, v)
        }
      }
    }
    // Row block i of G: for each input time j < i, block (i, j) is A^(i-j-1) B.
    for j in 0..<i {
      let blk = powers_ab[i - j - 1]
      for ii in 0..<n {
        for jj in 0..<m {
          let v = blk.at(ii, jj).unwrap_or(0.0)
          if v != 0.0 {
            g.set(i * n + ii, j * m + jj, v)
          }
        }
      }
    }
  }
  let w = @linear.Matrix::identity(h).kron(q)
  let r_tot = @linear.Matrix::identity(h).kron(r)
  let g_t = g.transpose()
  let h_mat = g_t.mat_mat(w).mat_mat(g).add(r_tot)
  {
    a,
    b,
    q,
    r,
    horizon: h,
    input_limit: input_limit.unwrap_or(0.0),
    refine_steps: refine_steps.unwrap_or(64),
    f,
    g,
    w,
    r_tot,
    hess: h_mat,
  }
}

///|
/// The optimal first control input for state `x` tracking `x_ref`.
pub fn Mpc::step(
  self : Mpc,
  x : @linear.Vector,
  x_ref : @linear.Vector,
) -> @linear.Vector {
  let n = x.dim()
  let m = self.b.cols
  let h = self.horizon
  // c = F x0 - x_ref (stacked), used in the gradient b = G^T W c.
  let fx = self.f.mat_vec(x)
  let c = @linear.Vector::zeros(n * h)
  for k in 0..<h {
    for i in 0..<n {
      c.set(
        k * n + i,
        fx.get(k * n + i).unwrap_or(0.0) - x_ref.get(i).unwrap_or(0.0),
      )
    }
  }
  let wc = self.w.mat_vec(c)
  let bvec = self.g.transpose().mat_vec(wc)
  // Unconstrained optimum: solve H U = -b.
  let u = self.hess.solve(bvec.scale(-1.0))
  if self.input_limit > 0.0 {
    self.refine(u, m * h, bvec)
  }
  let u0 = @linear.Vector::zeros(m)
  for i in 0..<m {
    u0.set(i, u.get(i).unwrap_or(0.0))
  }
  u0
}

///|
/// Projected gradient descent that keeps the input sequence inside the limits.
/// The cost is 0.5 U^T H U + b^T U, so its gradient is H U + b.
fn Mpc::refine(
  self : Mpc,
  u : @linear.Vector,
  dim : Int32,
  b : @linear.Vector,
) -> Unit {
  let step = self.step_size()
  for _ in 0..<self.refine_steps {
    let hu = self.hess.mat_vec(u)
    for i in 0..<dim {
      let g = hu.get(i).unwrap_or(0.0) + b.get(i).unwrap_or(0.0)
      let v = u.get(i).unwrap_or(0.0) - step * g
      u.set(i, clamp1(v, self.input_limit))
    }
  }
}

///|
fn clamp1(x : Double, limit : Double) -> Double {
  if x < -limit {
    -limit
  } else if x > limit {
    limit
  } else {
    x
  }
}

///|
/// A safe step size for the projected gradient method: the reciprocal of the
/// largest row sum of the Hessian (an upper bound on its spectral norm).
fn Mpc::step_size(self : Mpc) -> Double {
  let n = self.hess.rows
  let mut max_row = 1.0e-9
  for i in 0..<n {
    let mut s = 0.0
    for j in 0..<n {
      s += self.hess.data[i * n + j].abs()
    }
    if s > max_row {
      max_row = s
    }
  }
  0.9 / max_row
}
