///|
/// A dense matrix of `Double` in row-major order.
///
/// Element `(i, j)` lives at `data[i * cols + j]`. The arithmetic methods
/// return new matrices; `set` is the only writer.
#derive(Debug)
pub(all) struct Mat {
  rows : Int32
  cols : Int32
  data : FixedArray[Double]
}

///|
/// An all-zero `rows x cols` matrix.
pub fn Mat::zeros(rows : Int32, cols : Int32) -> Mat {
  { rows, cols, data: FixedArray::make(rows * cols, 0.0), }
}

///|
/// A `rows x cols` matrix with every entry `value`.
pub fn Mat::filled(rows : Int32, cols : Int32, value : Double) -> Mat {
  { rows, cols, data: FixedArray::make(rows * cols, value), }
}

///|
/// The `n x n` identity.
pub fn Mat::identity(n : Int32) -> Mat {
  let data = FixedArray::make(n * n, 0.0)
  for i in 0..<n {
    data[i * n + i] = 1.0
  }
  { rows: n, cols: n, data, }
}

///|
/// A diagonal matrix with `values` on the diagonal.
pub fn Mat::diagonal(values : Vec) -> Mat {
  let n = values.len()
  let data = FixedArray::make(n * n, 0.0)
  for i in 0..<n {
    data[i * n + i] = values.get(i)
  }
  { rows: n, cols: n, data, }
}

///|
/// A matrix whose rows are `values`; every row must have the same length.
pub fn Mat::of(values : Array[Array[Double]]) -> Mat {
  let rows = values.length()
  if rows == 0 {
    return Mat::zeros(0, 0)
  }
  let cols = values[0].length()
  let data = FixedArray::make(rows * cols, 0.0)
  for i in 0..<rows {
    if values[i].length() != cols {
      abort("Mat::of: rows of uneven length")
    }
    for j in 0..<cols {
      data[i * cols + j] = values[i][j]
    }
  }
  { rows, cols, data, }
}

///|
/// A matrix that takes over `data` without copying it. `data` holds the
/// entries row by row.
pub fn Mat::from_fixed(
  rows : Int32,
  cols : Int32,
  data : FixedArray[Double],
) -> Mat {
  if data.length() != rows * cols {
    abort("Mat::from_fixed: storage does not match the shape")
  }
  { rows, cols, data, }
}

///|
/// The entry at `(i, j)`. An index outside the matrix aborts.
pub fn Mat::get(self : Mat, i : Int32, j : Int32) -> Double {
  self.data[i * self.cols + j]
}

///|
/// The entry at `(i, j)`, or `None` when the index is outside the matrix.
pub fn Mat::at(self : Mat, i : Int32, j : Int32) -> Double? {
  if i >= 0 && i < self.rows && j >= 0 && j < self.cols {
    Some(self.data[i * self.cols + j])
  } else {
    None
  }
}

///|
/// Overwrite the entry at `(i, j)`.
pub fn Mat::set(self : Mat, i : Int32, j : Int32, value : Double) -> Unit {
  self.data[i * self.cols + j] = value
}

///|
/// Add `value` to the entry at `(i, j)`.
pub fn Mat::add_to(self : Mat, i : Int32, j : Int32, value : Double) -> Unit {
  self.data[i * self.cols + j] += value
}

///|
/// The `i`-th row, copied out.
pub fn Mat::row(self : Mat, i : Int32) -> Vec {
  let data = FixedArray::make(self.cols, 0.0)
  for j in 0..<self.cols {
    data[j] = self.data[i * self.cols + j]
  }
  { data, }
}

///|
/// The `j`-th column, copied out.
pub fn Mat::col(self : Mat, j : Int32) -> Vec {
  let data = FixedArray::make(self.rows, 0.0)
  for i in 0..<self.rows {
    data[i] = self.data[i * self.cols + j]
  }
  { data, }
}

///|
/// Overwrite the `i`-th row with `values`.
pub fn Mat::set_row(self : Mat, i : Int32, values : Vec) -> Unit {
  if values.len() != self.cols {
    abort("Mat::set_row: length mismatch")
  }
  for j in 0..<self.cols {
    self.data[i * self.cols + j] = values.get(j)
  }
}

///|
/// Overwrite the `j`-th column with `values`.
pub fn Mat::set_col(self : Mat, j : Int32, values : Vec) -> Unit {
  if values.len() != self.rows {
    abort("Mat::set_col: length mismatch")
  }
  for i in 0..<self.rows {
    self.data[i * self.cols + j] = values.get(i)
  }
}

///|
/// The `height x width` block whose top-left corner is `(row, col)`.
pub fn Mat::block(
  self : Mat,
  row~ : Int32,
  col~ : Int32,
  height~ : Int32,
  width~ : Int32,
) -> Mat {
  if row < 0 ||
    col < 0 ||
    height < 0 ||
    width < 0 ||
    row + height > self.rows ||
    col + width > self.cols {
    abort("Mat::block: block outside the matrix")
  }
  let out = Mat::zeros(height, width)
  for i in 0..<height {
    for j in 0..<width {
      out.set(i, j, self.get(row + i, col + j))
    }
  }
  out
}

///|
/// The transpose `A^T`.
pub fn Mat::transpose(self : Mat) -> Mat {
  let data = FixedArray::make(self.rows * self.cols, 0.0)
  for i in 0..<self.rows {
    for j in 0..<self.cols {
      data[j * self.rows + i] = self.data[i * self.cols + j]
    }
  }
  { rows: self.cols, cols: self.rows, data, }
}

///|
/// Every entry multiplied by `factor`.
pub fn Mat::scale(self : Mat, factor : Double) -> Mat {
  let data = FixedArray::make(self.data.length(), 0.0)
  for k in 0..<self.data.length() {
    data[k] = self.data[k] * factor
  }
  { rows: self.rows, cols: self.cols, data, }
}

///|
/// The elementwise sum. Both matrices must have the same shape.
pub fn Mat::add(self : Mat, other : Mat) -> Mat {
  if self.rows != other.rows || self.cols != other.cols {
    abort("Mat::add: shape mismatch")
  }
  let data = FixedArray::make(self.data.length(), 0.0)
  for k in 0..<self.data.length() {
    data[k] = self.data[k] + other.data[k]
  }
  { rows: self.rows, cols: self.cols, data, }
}

///|
/// The elementwise difference. Both matrices must have the same shape.
pub fn Mat::sub(self : Mat, other : Mat) -> Mat {
  if self.rows != other.rows || self.cols != other.cols {
    abort("Mat::sub: shape mismatch")
  }
  let data = FixedArray::make(self.data.length(), 0.0)
  for k in 0..<self.data.length() {
    data[k] = self.data[k] - other.data[k]
  }
  { rows: self.rows, cols: self.cols, data, }
}

///|
/// Every entry negated.
pub fn Mat::negate(self : Mat) -> Mat {
  self.scale(-1.0)
}

///|
/// The product `self * other`.
pub fn Mat::mul(self : Mat, other : Mat) -> Mat {
  if self.cols != other.rows {
    abort("Mat::mul: shape mismatch")
  }
  let data = FixedArray::make(self.rows * other.cols, 0.0)
  for i in 0..<self.rows {
    for k in 0..<self.cols {
      let a = self.data[i * self.cols + k]
      if a != 0.0 {
        for j in 0..<other.cols {
          data[i * other.cols + j] += a * other.data[k * other.cols + j]
        }
      }
    }
  }
  { rows: self.rows, cols: other.cols, data, }
}

///|
/// The product `self * v`.
pub fn Mat::apply(self : Mat, v : Vec) -> Vec {
  if v.len() != self.cols {
    abort("Mat::apply: length mismatch")
  }
  let data = FixedArray::make(self.rows, 0.0)
  for i in 0..<self.rows {
    let mut sum = 0.0
    for j in 0..<self.cols {
      sum += self.data[i * self.cols + j] * v.get(j)
    }
    data[i] = sum
  }
  { data, }
}

///|
/// The product `v^T * self`, a row vector times the matrix.
pub fn Mat::apply_transposed(self : Mat, v : Vec) -> Vec {
  if v.len() != self.rows {
    abort("Mat::apply_transposed: length mismatch")
  }
  let data = FixedArray::make(self.cols, 0.0)
  for j in 0..<self.cols {
    let mut sum = 0.0
    for i in 0..<self.rows {
      sum += v.get(i) * self.data[i * self.cols + j]
    }
    data[j] = sum
  }
  { data, }
}

///|
/// The sum of the diagonal entries.
pub fn Mat::trace(self : Mat) -> Double {
  let n = if self.rows < self.cols { self.rows } else { self.cols }
  let mut total = 0.0
  for i in 0..<n {
    total += self.get(i, i)
  }
  total
}

///|
/// `self` with `right` appended on the right, so the result is
/// `rows x (cols + right.cols)`.
pub fn Mat::augment(self : Mat, right : Mat) -> Mat {
  if self.rows != right.rows {
    abort("Mat::augment: row count mismatch")
  }
  let out = Mat::zeros(self.rows, self.cols + right.cols)
  for i in 0..<self.rows {
    for j in 0..<self.cols {
      out.set(i, j, self.get(i, j))
    }
    for j in 0..<right.cols {
      out.set(i, self.cols + j, right.get(i, j))
    }
  }
  out
}

///|
/// `self` with `below` appended underneath, so the result is
/// `(rows + below.rows) x cols`.
pub fn Mat::stack(self : Mat, below : Mat) -> Mat {
  if self.cols != below.cols {
    abort("Mat::stack: column count mismatch")
  }
  let out = Mat::zeros(self.rows + below.rows, self.cols)
  for i in 0..<self.rows {
    for j in 0..<self.cols {
      out.set(i, j, self.get(i, j))
    }
  }
  for i in 0..<below.rows {
    for j in 0..<below.cols {
      out.set(self.rows + i, j, below.get(i, j))
    }
  }
  out
}

///|
/// Whether every entry is finite.
pub fn Mat::is_finite(self : Mat) -> Bool {
  for k in 0..<self.data.length() {
    if !self.data[k].is_finite() {
      return false
    }
  }
  true
}

///|
/// Whether `self` and `other` agree entry by entry within `tolerance`.
pub fn Mat::approx(
  self : Mat,
  other : Mat,
  tolerance~ : Double = 1.0e-9,
) -> Bool {
  if self.rows != other.rows || self.cols != other.cols {
    return false
  }
  for k in 0..<self.data.length() {
    if (self.data[k] - other.data[k]).abs() > tolerance {
      return false
    }
  }
  true
}

///|
/// The rows copied into growable arrays, for output and test expectations.
pub fn Mat::to_array_rows(self : Mat) -> Array[Array[Double]] {
  let out = Array::make(self.rows, Array::make(0, 0.0))
  for i in 0..<self.rows {
    out[i] = self.row(i).to_array()
  }
  out
}

///|
/// One line per row, each formatted like a `Vec`.
pub fn Mat::to_string(self : Mat) -> String {
  let lines = Array::make(self.rows, "")
  for i in 0..<self.rows {
    lines[i] = self.row(i).to_string()
  }
  lines.join("\n")
}

///|
pub extend Mat with @debug.Debug::{to_repr}
