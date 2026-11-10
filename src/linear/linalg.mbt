// The free functions below take the names `numpy.linalg` gives the same
// operations, so a reader coming from that vocabulary finds them where they
// expect.

///|
/// Solve `a * x = b` for a square `a`.
pub fn solve(a : Mat, b : Mat) -> Mat raise Refused {
  lu(a).solve(b)
}

///|
/// Solve `a * x = b` for a square `a` and a single right-hand side.
pub fn solve_vec(a : Mat, b : Vec) -> Vec raise Refused {
  if b.len() != a.rows {
    raise Refused::Dimension(got=b.len(), want=a.rows)
  }
  let rhs = Mat::zeros(b.len(), 1)
  for i in 0..<b.len() {
    rhs.set(i, 0, b.get(i))
  }
  lu(a).solve(rhs).col(0)
}

///|
/// The inverse of a square matrix.
pub fn inv(a : Mat) -> Mat raise Refused {
  if a.rows != a.cols {
    raise Refused::Dimension(got=a.cols, want=a.rows)
  }
  lu(a).solve(Mat::identity(a.rows))
}

///|
/// The determinant of a square matrix.
pub fn det(a : Mat) -> Double raise Refused {
  lu(a).det()
}

///|
/// The Frobenius norm of a matrix.
pub fn norm(a : Mat) -> Double {
  let mut sum = 0.0
  for k in 0..<a.data.length() {
    sum += a.data[k] * a.data[k]
  }
  sum.sqrt()
}
