///|
/// The matrix exponential of a square matrix, by scaling and squaring followed
/// by a truncated Taylor series.
///
/// This is what turns a continuous state matrix into a discrete one; the
/// scaling keeps the series inside the range where twenty terms reach machine
/// precision.
pub fn expm(a : Mat, terms~ : Int32 = 20) -> Mat raise Refused {
  let n = a.rows
  if a.rows != a.cols {
    raise Refused::Dimension(got=a.cols, want=a.rows)
  }
  if n == 0 {
    raise Refused::Empty
  }
  let mut norm = 0.0
  for i in 0..<n {
    let mut row_sum = 0.0
    for j in 0..<n {
      row_sum += a.get(i, j).abs()
    }
    if row_sum > norm {
      norm = row_sum
    }
  }
  let mut scale = 1.0
  let mut squarings = 0
  while norm / scale > 0.5 {
    scale *= 2.0
    squarings += 1
  }
  let scaled = a.scale(1.0 / scale)
  let mut result = Mat::identity(n)
  let mut term = Mat::identity(n)
  for k in 1..<=terms {
    term = scaled.mul(term).scale(1.0 / k.to_double())
    result = result.add(term)
  }
  let mut out = result
  for _ in 0..<squarings {
    out = out.mul(out)
  }
  out
}
