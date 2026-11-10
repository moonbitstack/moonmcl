///|
/// The lower Cholesky factor of a symmetric positive-definite matrix `a`, so
/// that `a = l * l^T`.
///
/// The factor is what the Kalman gain is solved with: a Cholesky solve costs
/// half an LU solve and never needs pivoting for a covariance.
pub fn cholesky(a : Mat) -> Mat raise Refused {
  let n = a.rows
  if a.rows != a.cols {
    raise Refused::Dimension(got=a.cols, want=a.rows)
  }
  if n == 0 {
    raise Refused::Empty
  }
  let l = Mat::zeros(n, n)
  for j in 0..<n {
    let mut sum = a.get(j, j)
    for k in 0..<j {
      sum -= l.get(j, k) * l.get(j, k)
    }
    if sum <= 0.0 {
      raise Refused::NotPositiveDefinite(at=j)
    }
    let pivot = sum.sqrt()
    l.set(j, j, pivot)
    for i in (j + 1)..<n {
      let mut s = a.get(i, j)
      for k in 0..<j {
        s -= l.get(i, k) * l.get(j, k)
      }
      l.set(i, j, s / pivot)
    }
  }
  l
}

///|
/// Solve `a * x = b` given the lower Cholesky factor `l` of `a`. `b` has one
/// column per right-hand side.
pub fn cholesky_solve(l : Mat, b : Mat) -> Mat raise Refused {
  let n = l.rows
  if b.rows != n {
    raise Refused::Dimension(got=b.rows, want=n)
  }
  let width = b.cols
  let y = Mat::zeros(n, width)
  for i in 0..<n {
    for j in 0..<width {
      let mut sum = b.get(i, j)
      for k in 0..<i {
        sum -= l.get(i, k) * y.get(k, j)
      }
      y.set(i, j, sum / l.get(i, i))
    }
  }
  for i in (n - 1)>=..0 {
    for j in 0..<width {
      let mut sum = y.get(i, j)
      for k in (i + 1)..<n {
        sum -= l.get(k, i) * y.get(k, j)
      }
      y.set(i, j, sum / l.get(i, i))
    }
  }
  y
}
