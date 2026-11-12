// Dense linear algebra against a mature implementation. The table comes from
// `scripts/vectors/gen_linear.py`, which asks NumPy and SciPy for each result;
// `moon test` reads only the table, so nobody needs those packages installed.
//
// Sign conventions are the one place a factorisation is not unique: QR is fixed
// here the same way the generator fixed it, by making the largest entry of every
// column of Q positive and flipping the matching row of R.

///|
fn replay_mat(flat : Array[Double], n : Int32) -> @linear.Mat {
  let matrix = @linear.Mat::zeros(n, n)
  for i in 0..<n {
    for j in 0..<n {
      matrix.set(i, j, flat[i * n + j])
    }
  }
  matrix
}

///|
fn replay_close(
  matrix : @linear.Mat,
  expected : Array[Double],
  n~ : Int32,
  tolerance~ : Double = 1.0e-9,
) -> Bool {
  for i in 0..<n {
    for j in 0..<n {
      if (matrix.get(i, j) - expected[i * n + j]).abs() > tolerance {
        return false
      }
    }
  }
  true
}

///|
fn replay_vec_close(
  vector : @linear.Vec,
  expected : Array[Double],
  tolerance~ : Double = 1.0e-9,
) -> Bool {
  for i in 0..<expected.length() {
    if (vector.get(i) - expected[i]).abs() > tolerance {
      return false
    }
  }
  true
}

///|
/// Make each column of `q` point the way its largest entry says, and flip the
/// matching row of `r` to keep the product unchanged.
fn orient_qr(q : @linear.Mat, r : @linear.Mat, n : Int32) -> Unit {
  for j in 0..<n {
    let mut pivot = 0
    for i in 1..<q.rows {
      if q.get(i, j).abs() > q.get(pivot, j).abs() {
        pivot = i
      }
    }
    if q.get(pivot, j) < 0.0 {
      for i in 0..<q.rows {
        q.set(i, j, -q.get(i, j))
      }
      for k in 0..<n {
        r.set(j, k, -r.get(j, k))
      }
    }
  }
  ignore(n)
}

///|
test "solve, inverse and determinant match the reference" {
  let cases = linalg_cases()
  assert_eq(cases.length() >= 9, true)
  for case in cases {
    let a = replay_mat(case.a, case.n)
    let b = @linear.Vec::of(case.b)
    assert_eq(replay_vec_close(@linear.solve_vec(a, b), case.solution), true)
    assert_eq(replay_close(@linear.inv(a), case.inverse, n=case.n), true)
    assert_eq((@linear.det(a) - case.determinant).abs() < 1.0e-9, true)
    assert_eq((@linear.lu(a).det() - case.determinant).abs() < 1.0e-9, true)
  }
}

///|
test "the Cholesky factor and the symmetric eigenvalues match the reference" {
  for case in linalg_cases() {
    let spd = replay_mat(case.spd, case.n)
    let factor = @linear.cholesky(spd)
    assert_eq(replay_close(factor, case.cholesky, n=case.n), true)
    // Only the eigenvalues are compared: an eigenvector is fixed up to sign,
    // and the reconstruction is checked in the package's own tests.
    let symmetric = replay_mat(case.sym, case.n)
    let spectrum = @linear.eigh(symmetric)
    assert_eq(
      replay_vec_close(spectrum.values, case.eigenvalues, tolerance=1.0e-9),
      true,
    )
  }
}

///|
test "the matrix exponential matches the reference" {
  for case in linalg_cases() {
    let small = replay_mat(case.small, case.n)
    // Scaling and squaring followed by twenty Taylor terms reaches the same
    // digits as the reference on matrices of this size.
    assert_eq(
      replay_close(
        @linear.expm(small),
        case.exponential,
        tolerance=1.0e-9,
        n=case.n,
      ),
      true,
    )
  }
}

///|
test "the QR factors match the reference up to column signs" {
  for case in linalg_cases() {
    let a = replay_mat(case.a, case.n)
    let factor = @linear.qr(a)
    orient_qr(factor.q, factor.r, case.n)
    assert_eq(replay_close(factor.q, case.q, tolerance=1.0e-10, n=case.n), true)
    assert_eq(replay_close(factor.r, case.r, tolerance=1.0e-10, n=case.n), true)
  }
}
