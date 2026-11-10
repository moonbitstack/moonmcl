///|
/// An LU factorisation with partial pivoting, `perm * A = lower * upper`, where
/// `lower` is unit lower triangular and `upper` is upper triangular.
///
/// `sign` is the parity of the row swaps, so `det = sign * product(upper)`. The
/// factorisation is kept so that several right-hand sides can reuse it, which
/// is what the estimators do inside a filter loop.
#derive(Debug)
pub(all) struct Lu {
  n : Int32
  lower : Mat
  upper : Mat
  perm : Array[Int32]
  sign : Double
}

///|
/// Factorise `a`, a square matrix, into `perm * a = lower * upper`.
///
/// Raises `Refused::Singular` at the first column whose pivot is zero;
/// `Refused::Dimension` when `a` is not square.
pub fn lu(a : Mat) -> Lu raise Refused {
  let n = a.rows
  if a.rows != a.cols {
    raise Refused::Dimension(got=a.cols, want=a.rows)
  }
  if n == 0 {
    raise Refused::Empty
  }
  let work = Mat::zeros(n, n)
  for i in 0..<n {
    for j in 0..<n {
      work.set(i, j, a.get(i, j))
    }
  }
  let perm = Array::make(n, 0)
  for i in 0..<n {
    perm[i] = i
  }
  let mut sign = 1.0
  for k in 0..<n {
    let mut pivot_row = k
    let mut best = work.get(k, k).abs()
    for i in (k + 1)..<n {
      let v = work.get(i, k).abs()
      if v > best {
        best = v
        pivot_row = i
      }
    }
    if best == 0.0 {
      raise Refused::Singular(at=k)
    }
    if pivot_row != k {
      for j in 0..<n {
        let t = work.get(k, j)
        work.set(k, j, work.get(pivot_row, j))
        work.set(pivot_row, j, t)
      }
      let t = perm[k]
      perm[k] = perm[pivot_row]
      perm[pivot_row] = t
      sign = -sign
    }
    let pivot = work.get(k, k)
    for i in (k + 1)..<n {
      let factor = work.get(i, k) / pivot
      work.set(i, k, factor)
      for j in (k + 1)..<n {
        work.set(i, j, work.get(i, j) - factor * work.get(k, j))
      }
    }
  }
  let lower = Mat::zeros(n, n)
  let upper = Mat::zeros(n, n)
  for i in 0..<n {
    lower.set(i, i, 1.0)
    for j in 0..<i {
      lower.set(i, j, work.get(i, j))
    }
    for j in i..<n {
      upper.set(i, j, work.get(i, j))
    }
  }
  { n, lower, upper, perm, sign, }
}

///|
/// Solve `a * x = b` from a factorisation. `b` has one column per right-hand
/// side; the result has the same width.
pub fn Lu::solve(self : Lu, b : Mat) -> Mat raise Refused {
  if b.rows != self.n {
    raise Refused::Dimension(got=b.rows, want=self.n)
  }
  let width = b.cols
  let y = Mat::zeros(self.n, width)
  for i in 0..<self.n {
    for j in 0..<width {
      y.set(i, j, b.get(self.perm[i], j))
    }
  }
  for i in 0..<self.n {
    for j in 0..<width {
      let mut sum = y.get(i, j)
      for k in 0..<i {
        sum -= self.lower.get(i, k) * y.get(k, j)
      }
      y.set(i, j, sum)
    }
  }
  for i in (self.n - 1)>=..0 {
    for j in 0..<width {
      let mut sum = y.get(i, j)
      for k in (i + 1)..<self.n {
        sum -= self.upper.get(i, k) * y.get(k, j)
      }
      y.set(i, j, sum / self.upper.get(i, i))
    }
  }
  y
}

///|
/// The determinant of the factorised matrix.
pub fn Lu::det(self : Lu) -> Double {
  let mut product = self.sign
  for i in 0..<self.n {
    product *= self.upper.get(i, i)
  }
  product
}

///|
pub extend Lu with @debug.Debug::{to_repr}
