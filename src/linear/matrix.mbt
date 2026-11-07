// A row-major matrix of Double. Storage is `rows * cols` doubles; element
// (i, j) lives at `data[i * cols + j]`. The products, transposes and solvers
// below are written in place on plain arrays to keep the hot loops allocation
// free.

///|
#derive(@debug.Debug)
pub struct Matrix {
  data : Array[Double]
  rows : Int32
  cols : Int32
}

///|
pub extend Matrix with @debug.Debug::{to_repr}

///|
/// An all-zero `rows x cols` matrix.
pub fn Matrix::zeros(rows : Int32, cols : Int32) -> Matrix {
  { data: Array::makei(rows * cols, _ => 0.0), rows, cols, }
}

///|
/// A matrix whose rows are the given arrays.
pub fn Matrix::of(rows : Array[Array[Double]]) -> Matrix {
  guard rows is [first, ..] else { return Matrix::zeros(0, 0) }
  let cols = first.length()
  let data = Array(capacity=rows.length() * cols)
  for r in rows.iter() {
    guard r.length() == cols else { abort("Matrix::of: rows of uneven length") }
    for x in r.iter() {
      data.push(x)
    }
  }
  { data, rows: rows.length(), cols, }
}

///|
/// A matrix whose columns are the given arrays.
pub fn Matrix::of_col(cols : Array[Array[Double]]) -> Matrix {
  guard cols is [first, ..] else { return Matrix::zeros(0, 0) }
  let rows = first.length()
  let data = Array::makei(rows * cols.length(), _ => 0.0)
  for c in 0..<cols.length() {
    guard cols[c].length() == rows else {
      abort("Matrix::of_col: columns of uneven length")
    }
    for r in 0..<rows {
      data[r * cols.length() + c] = cols[c][r]
    }
  }
  { data, rows, cols: cols.length(), }
}

///|
/// The `n x n` identity.
pub fn Matrix::identity(n : Int32) -> Matrix {
  let data = Array::makei(n * n, _ => 0.0)
  for i in 0..<n {
    data[i * n + i] = 1.0
  }
  { data, rows: n, cols: n, }
}

///|
/// The element at (i, j) as a `Some`, or `None` when out of range.
pub fn Matrix::at(self : Matrix, i : Int32, j : Int32) -> Double? {
  if 0 <= i && i < self.rows && 0 <= j && j < self.cols {
    Some(self.data[i * self.cols + j])
  } else {
    None
  }
}

///|
pub fn Matrix::set(self : Matrix, i : Int32, j : Int32, v : Double) -> Unit {
  self.data[i * self.cols + j] = v
}

///|
/// The `i`-th row, copied into a vector.
pub fn Matrix::row(self : Matrix, i : Int32) -> Vector {
  let v = { data: Array(capacity=self.cols), }
  for j in 0..<self.cols {
    v.data.push(self.data[i * self.cols + j])
  }
  v
}

///|
/// The `j`-th column, copied into a vector.
pub fn Matrix::col(self : Matrix, j : Int32) -> Vector {
  let v = { data: Array(capacity=self.rows), }
  for i in 0..<self.rows {
    v.data.push(self.data[i * self.cols + j])
  }
  v
}

///|
pub fn Matrix::transpose(self : Matrix) -> Matrix {
  let data = Array(capacity=self.rows * self.cols)
  for j in 0..<self.cols {
    for i in 0..<self.rows {
      data.push(self.data[i * self.cols + j])
    }
  }
  { data, rows: self.cols, cols: self.rows, }
}

///|
pub fn Matrix::scale(self : Matrix, s : Double) -> Matrix {
  {
    data: [
      for x in self.data.iter() => x * s
    ],
    rows: self.rows,
    cols: self.cols,
  }
}

///|
pub fn Matrix::add(self : Matrix, other : Matrix) -> Matrix {
  guard self.rows == other.rows && self.cols == other.cols else {
    abort("Matrix::add: shape mismatch")
  }
  {
    data: [
      for k in 0..<self.data.length() => self.data[k] + other.data[k]
    ],
    rows: self.rows,
    cols: self.cols,
  }
}

///|
pub fn Matrix::sub(self : Matrix, other : Matrix) -> Matrix {
  guard self.rows == other.rows && self.cols == other.cols else {
    abort("Matrix::sub: shape mismatch")
  }
  {
    data: [
      for k in 0..<self.data.length() => self.data[k] - other.data[k]
    ],
    rows: self.rows,
    cols: self.cols,
  }
}

///|
/// `self * v`: a `rows x cols` matrix times a `cols`-long vector.
pub fn Matrix::mat_vec(self : Matrix, v : Vector) -> Vector {
  guard v.dim() == self.cols else { abort("Matrix::mat_vec: length mismatch") }
  let out = { data: Array(capacity=self.rows), }
  for i in 0..<self.rows {
    let base = i * self.cols
    let mut s = 0.0
    for j in 0..<self.cols {
      s += self.data[base + j] * v.data[j]
    }
    out.data.push(s)
  }
  out
}

///|
/// `self * b`: a `rows x k` matrix times a `k x cols` matrix.
pub fn Matrix::mat_mat(self : Matrix, b : Matrix) -> Matrix {
  guard self.cols == b.rows else { abort("Matrix::mat_mat: shape mismatch") }
  let data = Array::makei(self.rows * b.cols, _ => 0.0)
  for i in 0..<self.rows {
    for k in 0..<self.cols {
      let aik = self.data[i * self.cols + k]
      if aik != 0.0 {
        for j in 0..<b.cols {
          data[i * b.cols + j] += aik * b.data[k * b.cols + j]
        }
      }
    }
  }
  { data, rows: self.rows, cols: b.cols, }
}

///|
/// Solve `self * x = b` for `x`, where `self` is square. A dense LU
/// factorisation with partial pivoting; aborts if the matrix is singular.
pub fn Matrix::solve(self : Matrix, b : Vector) -> Vector {
  guard self.rows == self.cols else {
    abort("Matrix::solve: square matrix required")
  }
  let n = self.rows
  guard b.dim() == n else { abort("Matrix::solve: length mismatch") }
  let a = self.data.copy()
  let x = b.data.copy()
  for k in 0..<(n - 1) {
    let mut max_i = k
    let mut maxv = a[k * n + k].abs()
    for i in (k + 1)..<n {
      let v = a[i * n + k].abs()
      if v > maxv {
        maxv = v
        max_i = i
      }
    }
    guard maxv > 0.0 else { abort("Matrix::solve: singular matrix") }
    if max_i != k {
      for j in 0..<n {
        let t = a[k * n + j]
        a[k * n + j] = a[max_i * n + j]
        a[max_i * n + j] = t
      }
      let t = x[k]
      x[k] = x[max_i]
      x[max_i] = t
    }
    let pivot = a[k * n + k]
    for i in (k + 1)..<n {
      let f = a[i * n + k] / pivot
      a[i * n + k] = f
      for j in (k + 1)..<n {
        a[i * n + j] -= f * a[k * n + j]
      }
      x[i] -= f * x[k]
    }
  }
  for i in (n - 1)>=..0 {
    let mut s = x[i]
    for j in (i + 1)..<n {
      s -= a[i * n + j] * x[j]
    }
    x[i] = s / a[i * n + i]
  }
  { data: x, }
}

///|
/// An economy QR factorisation by modified Gram-Schmidt, `self = Q * R`, with
/// `Q` of orthonormal columns and `R` upper triangular. Returns `(Q, R)`.
pub fn Matrix::qr(self : Matrix) -> (Matrix, Matrix) {
  let m = self.rows
  let n = self.cols
  let a = self.data.copy()
  let q = Matrix::zeros(m, n)
  let r = Matrix::zeros(n, n)
  for j in 0..<n {
    let vj = { data: Array::makei(m, i => a[i * n + j]), }
    for k in 0..<j {
      let qk = q.col(k)
      let rjk = vj.dot(qk)
      r.set(k, j, rjk)
      for i in 0..<m {
        vj.data[i] -= rjk * qk.data[i]
      }
    }
    let rjj = vj.norm()
    r.set(j, j, rjj)
    if rjj > 0.0 {
      for i in 0..<m {
        q.set(i, j, vj.data[i] / rjj)
      }
    }
  }
  (q, r)
}

///|
/// The inverse of a square matrix, computed by solving `self * X = I` column
/// by column with the LU solver; aborts if the matrix is singular.
pub fn Matrix::inv(self : Matrix) -> Matrix {
  guard self.rows == self.cols else {
    abort("Matrix::inv: square matrix required")
  }
  let n = self.rows
  let x = Matrix::zeros(n, n)
  for j in 0..<n {
    let e = Matrix::identity(n).col(j)
    let col = self.solve(e)
    for i in 0..<n {
      x.set(i, j, col.get(i).unwrap_or(0.0))
    }
  }
  x
}

///|
/// The Kronecker product of `self` (`m x n`) and `b` (`p x q`), an
/// `mp x nq` block matrix.
pub fn Matrix::kron(self : Matrix, b : Matrix) -> Matrix {
  let m = self.rows
  let n = self.cols
  let p = b.rows
  let q = b.cols
  let data = Array::makei(m * p * n * q, _ => 0.0)
  for i in 0..<m {
    for j in 0..<n {
      let a = self.data[i * n + j]
      if a != 0.0 {
        for ii in 0..<p {
          for jj in 0..<q {
            data[(i * p + ii) * n * q + j * q + jj] = a * b.data[ii * q + jj]
          }
        }
      }
    }
  }
  { data, rows: m * p, cols: n * q, }
}

///|
pub fn Matrix::to_string(self : Matrix) -> String {
  let lines = [ for i in 0..<self.rows => self.row(i).to_string() ]
  lines.join("\n")
}
