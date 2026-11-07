// Integral LQR: LQR with integral action on the output, which guarantees
// zero steady-state error to a constant reference. For the discrete plant
//   x_{k+1} = A x_k + B u_k,  y_k = C x_k
// the integral of the tracking error e_{k+1} = e_k + dt (r - C x_k) is
// appended to the state, giving the deviation dynamics
//   [ dx_{k+1} ]   [ A    0    ] [ dx_k ]   [ B ]
//   [ de_{k+1} ] = [ -dtC   1  ] [ de_k ] + [ 0 ] (u_k - u_s)
// whose LQR gain K is applied as u = u_s - K [x - x_s; e]. The equilibrium
// (x_s, u_s) satisfies C x_s = r and (I - A) x_s = B u_s.

///|
/// LQR with integral action on the output: the tracking error is appended to
/// the state, so a constant reference is reached with no steady-state offset.
#derive(@debug.Debug)
pub struct IntegralLqr {
  // Augmented gain K, size m x (n + m) ... here n + 1 (one integral per output).
  k : @linear.Matrix
  // Plant.
  a : @linear.Matrix
  b : @linear.Matrix
  c : @linear.Matrix
  // Reference the controller tracks (set at construction).
  reference : @linear.Vector
  // The discretisation period used in the integral augmentation.
  dt : Double
}

///|
pub extend IntegralLqr with @debug.Debug::{to_repr}

///|
/// Build the integral-augmented plant and its LQR gain. `reference` is the
/// constant setpoint the controller is designed to track; `dt` is the
/// discretisation period.
pub fn integral_lqr(
  a : @linear.Matrix,
  b : @linear.Matrix,
  c : @linear.Matrix,
  q : @linear.Matrix,
  q_int : @linear.Matrix,
  r : @linear.Matrix,
  reference : @linear.Vector,
  dt : Double,
) -> IntegralLqr {
  let n = a.rows
  let m = c.rows
  // A_bar = [ A  0 ; -dt C  1 ].
  let a_bar = @linear.Matrix::zeros(n + m, n + m)
  for i in 0..<n {
    for j in 0..<n {
      a_bar.set(i, j, a.at(i, j).unwrap_or(0.0))
    }
  }
  for i in 0..<m {
    for j in 0..<n {
      a_bar.set(n + i, j, -dt * c.at(i, j).unwrap_or(0.0))
    }
    a_bar.set(n + i, n + i, 1.0)
  }
  // B_bar = [ B ; 0 ].
  let b_bar = @linear.Matrix::zeros(n + m, b.cols)
  for i in 0..<n {
    for j in 0..<b.cols {
      b_bar.set(i, j, b.at(i, j).unwrap_or(0.0))
    }
  }
  // Q_bar = diag(Q, q_int).
  let q_bar = @linear.Matrix::zeros(n + m, n + m)
  for i in 0..<n {
    for j in 0..<n {
      q_bar.set(i, j, q.at(i, j).unwrap_or(0.0))
    }
  }
  for i in 0..<m {
    for j in 0..<m {
      q_bar.set(n + i, n + j, q_int.at(i, j).unwrap_or(0.0))
    }
  }
  let k = lqr_gain(a_bar, b_bar, q_bar, r)
  { k, a, b, c, reference, dt, }
}

///|
/// The augmented gain K, size m x (n + m).
pub fn IntegralLqr::gain(self : IntegralLqr) -> @linear.Matrix {
  self.k
}

///|
/// The control input for state `x` and integral state `e`:
/// u = u_s - K [x - x_s; e], with (x_s, u_s) the equilibrium for the stored
/// reference.
pub fn IntegralLqr::step(
  self : IntegralLqr,
  x : @linear.Vector,
  e : @linear.Vector,
) -> @linear.Vector {
  let n = x.dim()
  let m = self.reference.dim()
  // Equilibrium: (I - A) x_s - B u_s = 0,  C x_s = r. Solved as the square
  // (n+m) x (n+m) system  [ (I-A)  -B ; C  0 ] [x_s; u_s] = [0; r].
  let i_mat = @linear.Matrix::identity(n)
  let lhs_mat = i_mat.sub(self.a)
  let mmat = @linear.Matrix::zeros(n + m, n + m)
  for i in 0..<n {
    for j in 0..<n {
      mmat.set(i, j, lhs_mat.at(i, j).unwrap_or(0.0))
    }
    for j in 0..<m {
      mmat.set(i, n + j, -self.b.at(i, j).unwrap_or(0.0))
    }
  }
  for i in 0..<m {
    for j in 0..<n {
      mmat.set(n + i, j, self.c.at(i, j).unwrap_or(0.0))
    }
  }
  let rhs = @linear.Vector::zeros(n + m)
  for i in 0..<m {
    rhs.set(n + i, self.reference.get(i).unwrap_or(0.0))
  }
  let eq = mmat.solve(rhs)
  let x_s = @linear.Vector::zeros(n)
  let u_s = @linear.Vector::zeros(m)
  for i in 0..<n {
    x_s.set(i, eq.get(i).unwrap_or(0.0))
  }
  for i in 0..<m {
    u_s.set(i, eq.get(n + i).unwrap_or(0.0))
  }
  // Deviation state [x - x_s; e].
  let dev = @linear.Vector::zeros(n + m)
  for i in 0..<n {
    dev.set(i, x.get(i).unwrap_or(0.0) - x_s.get(i).unwrap_or(0.0))
  }
  for i in 0..<m {
    dev.set(n + i, e.get(i).unwrap_or(0.0))
  }
  let feedback = self.k.mat_vec(dev)
  let u = @linear.Vector::zeros(m)
  for i in 0..<m {
    u.set(i, u_s.get(i).unwrap_or(0.0) - feedback.get(i).unwrap_or(0.0))
  }
  u
}
