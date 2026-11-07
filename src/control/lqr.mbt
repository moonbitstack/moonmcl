// The linear quadratic regulator. Given a discrete-time linear system
//   x_{k+1} = A x_k + B u_k
// and cost weights Q (state) and R (input), `lqr_gain` solves the discrete
// algebraic Riccati equation by value iteration and returns the static state
// feedback gain K such that u = -K x minimises the infinite-horizon cost.

///|
/// The discrete algebraic Riccati equation solved by value iteration
///   P' = A^T P A - A^T P B (R + B^T P B)^-1 B^T P A + Q
/// starting from P = Q; returns the optimal cost-to-go matrix P.
pub fn dare(
  a : @linear.Matrix,
  b : @linear.Matrix,
  q : @linear.Matrix,
  r : @linear.Matrix,
  tolerance~ : Double? = Some(1.0e-12),
  max_iterations~ : Int32? = Some(100000),
) -> @linear.Matrix {
  let n = a.rows
  guard b.rows == n else { abort("dare: a and b rows must match") }
  guard q.rows == n && q.cols == n else { abort("dare: q must be n x n") }
  guard r.rows == b.cols && r.cols == b.cols else {
    abort("dare: r must be m x m")
  }
  let tol = tolerance.unwrap_or(1.0e-12)
  let max_iter = max_iterations.unwrap_or(100000)
  let a_t = a.transpose()
  let b_t = b.transpose()
  let mut p = q
  for _ in 0..<max_iter {
    let pb = p.mat_mat(b)
    let btp = b_t.mat_mat(p)
    let s = r.add(btp.mat_mat(b)).inv()
    let btpa = btp.mat_mat(a)
    let apb = a_t.mat_mat(pb)
    let middle = apb.mat_mat(s).mat_mat(btpa)
    let first = a_t.mat_mat(p).mat_mat(a)
    let p_next = first.sub(middle).add(q)
    if matrix_close(p, p_next, tol) {
      p = p_next
      break
    }
    p = p_next
  }
  p
}

///|
/// The optimal static state feedback gain K = (R + B^T P B)^-1 B^T P A for the
/// closed loop u = -K x.
pub fn lqr_gain(
  a : @linear.Matrix,
  b : @linear.Matrix,
  q : @linear.Matrix,
  r : @linear.Matrix,
  tolerance~ : Double? = Some(1.0e-12),
  max_iterations~ : Int32? = Some(100000),
) -> @linear.Matrix {
  let p = dare(a, b, q, r, tolerance~, max_iterations~)
  let b_t = b.transpose()
  let s = r.add(b_t.mat_mat(p).mat_mat(b)).inv()
  s.mat_mat(b_t.mat_mat(p).mat_mat(a))
}

///|
fn matrix_close(x : @linear.Matrix, y : @linear.Matrix, tol : Double) -> Bool {
  let n = x.data.length()
  for k in 0..<n {
    let d = (x.data[k] - y.data[k]).abs()
    if d > tol {
      return false
    }
  }
  true
}
