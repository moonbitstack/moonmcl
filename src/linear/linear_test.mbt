// Tests for the dense linear algebra: vectors, matrix products, the
// factorisations and the exponential. Each factorisation is checked against
// the identity it promises, which is the property the estimators rely on.

///|
fn refused(f : () -> Unit raise @linear.Refused) -> @linear.Refused? {
  try {
    f()
    None
  } catch {
    e => Some(e)
  }
}

///|
test "vec basic arithmetic" {
  let a = @linear.Vec::of([1.0, 2.0, 3.0])
  let b = @linear.Vec::of([4.0, 5.0, 6.0])
  assert_eq(a.add(b).to_array(), [5.0, 7.0, 9.0])
  assert_eq(a.sub(b).to_array(), [-3.0, -3.0, -3.0])
  assert_eq(a.hadamard(b).to_array(), [4.0, 10.0, 18.0])
  assert_eq(a.dot(b), 32.0)
  assert_eq(a.scale(2.0).sum(), 12.0)
  assert_eq(a.axpy(2.0, b).to_array(), [9.0, 12.0, 15.0])
  assert_eq(a.negate().sum(), -6.0)
}

///|
test "vec norm and normalisation" {
  let a = @linear.Vec::of([3.0, 4.0])
  assert_eq(a.norm(), 5.0)
  assert_eq(a.norm_squared(), 25.0)
  let n = a.normalized()
  assert_eq(n.approx(@linear.Vec::of([0.6, 0.8])), true)
  assert_eq(@linear.Vec::zeros(3).normalized().norm(), 0.0)
}

///|
test "vec accessors and copy" {
  let a = @linear.Vec::zeros(3)
  a.set(1, 2.5)
  assert_eq(a.get(1), 2.5)
  assert_eq(a.at(1), Some(2.5))
  assert_eq(a.at(3), None)
  let b = a.clone()
  b.set(1, 0.0)
  assert_eq(a.get(1), 2.5)
  assert_eq(a.max_abs(), 2.5)
  assert_eq(a.is_finite(), true)
  assert_eq(a.to_string(), "[ 0, 2.5, 0 ]")
}

///|
test "vec linspace" {
  let a = @linear.Vec::linspace(0.0, 1.0, 5)
  assert_eq(a.to_array(), [0.0, 0.25, 0.5, 0.75, 1.0])
  assert_eq(@linear.Vec::linspace(2.0, 4.0, 1).to_array(), [2.0])
}

///|
test "mat build and transpose" {
  let m = @linear.Mat::of([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
  assert_eq(m.rows, 3)
  assert_eq(m.cols, 2)
  assert_eq(m.transpose().rows, 2)
  assert_eq(m.transpose().cols, 3)
  assert_eq(m.get(2, 1), 6.0)
  assert_eq(m.at(3, 0), None)
  assert_eq(m.row(0).to_array(), [1.0, 2.0])
  assert_eq(m.col(1).to_array(), [2.0, 4.0, 6.0])
  assert_eq(m.trace(), 5.0)
  let eye = @linear.Mat::identity(3)
  assert_eq(eye.get(1, 1), 1.0)
  assert_eq(eye.get(1, 0), 0.0)
}

///|
test "mat products" {
  let a = @linear.Mat::of([[1.0, 2.0], [3.0, 4.0]])
  let b = @linear.Mat::of([[5.0, 6.0], [7.0, 8.0]])
  assert_eq(a.mul(b).to_array_rows(), [[19.0, 22.0], [43.0, 50.0]])
  assert_eq(a.apply(@linear.Vec::of([1.0, 1.0])).to_array(), [3.0, 7.0])
  assert_eq(a.apply_transposed(@linear.Vec::of([1.0, 1.0])).to_array(), [
    4.0, 6.0,
  ])
  assert_eq(a.scale(2.0).sub(a).approx(a), true)
  assert_eq(@linear.Mat::diagonal(@linear.Vec::of([1.0, 2.0])).get(1, 1), 2.0)
}

///|
test "mat blocks and stacking" {
  let a = @linear.Mat::of([[1.0, 2.0], [3.0, 4.0]])
  assert_eq(a.augment(a).to_array_rows(), [
    [1.0, 2.0, 1.0, 2.0],
    [3.0, 4.0, 3.0, 4.0],
  ])
  assert_eq(a.stack(a).cols, 2)
  assert_eq(a.stack(a).rows, 4)
  assert_eq(a.block(row=1, col=0, height=1, width=2).to_array_rows(), [
    [3.0, 4.0],
  ])
}

///|
test "solve a known system" {
  let a = @linear.Mat::of([[3.0, 1.0], [1.0, 2.0]])
  let b = @linear.Mat::of([[9.0], [8.0]])
  let x = @linear.solve(a, b)
  assert_eq(x.to_array_rows(), [[2.0], [3.0]])
  assert_eq(@linear.solve_vec(a, @linear.Vec::of([9.0, 8.0])).to_array(), [
    2.0, 3.0,
  ])
}

///|
test "inverse and determinant" {
  let a = @linear.Mat::of([[4.0, 7.0], [2.0, 6.0]])
  let inverse = @linear.inv(a)
  assert_eq(
    inverse.approx(
      @linear.Mat::of([[0.6, -0.7], [-0.2, 0.4]]),
      tolerance=1.0e-12,
    ),
    true,
  )
  assert_eq((@linear.det(a) - 10.0).abs() < 1.0e-12, true)
  assert_eq(
    a.mul(inverse).approx(@linear.Mat::identity(2), tolerance=1.0e-12),
    true,
  )
}

///|
test "lu keeps the factorisation reusable" {
  let a = @linear.Mat::of([[2.0, 1.0, 1.0], [4.0, 3.0, 3.0], [8.0, 7.0, 9.0]])
  let f = @linear.lu(a)
  let b = @linear.Mat::of([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
  let x = f.solve(b)
  assert_eq(a.mul(x).approx(b, tolerance=1.0e-12), true)
  assert_eq((f.det() - 4.0).abs() < 1.0e-12, true)
  for i in 0..<3 {
    assert_eq(f.lower.get(i, i), 1.0)
  }
}

///|
test "qr factors into orthonormal times triangular" {
  let a = @linear.Mat::of([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
  let f = @linear.qr(a)
  assert_eq(f.q.rows, 3)
  assert_eq(f.q.cols, 2)
  assert_eq(f.r.rows, 2)
  assert_eq(f.r.get(1, 0), 0.0)
  assert_eq(
    f.q.transpose().mul(f.q).approx(@linear.Mat::identity(2), tolerance=1.0e-12),
    true,
  )
  assert_eq(f.q.mul(f.r).approx(a, tolerance=1.0e-12), true)
}

///|
test "cholesky of a positive definite matrix" {
  let a = @linear.Mat::of([[4.0, 2.0], [2.0, 3.0]])
  let l = @linear.cholesky(a)
  assert_eq(l.mul(l.transpose()).approx(a, tolerance=1.0e-12), true)
  let b = @linear.Mat::of([[6.0], [7.0]])
  let x = @linear.cholesky_solve(l, b)
  assert_eq(a.mul(x).approx(b, tolerance=1.0e-12), true)
}

///|
test "cholesky refuses a matrix that is not positive definite" {
  let a = @linear.Mat::of([[1.0, 2.0], [2.0, 1.0]])
  match refused(fn() raise @linear.Refused { ignore(@linear.cholesky(a)) }) {
    Some(@linear.Refused::NotPositiveDefinite(at~)) => assert_eq(at, 1)
    _ => abort("expected NotPositiveDefinite")
  }
}

///|
test "lu refuses a singular matrix and a ragged shape" {
  let a = @linear.Mat::of([[1.0, 2.0], [2.0, 4.0]])
  match refused(fn() raise @linear.Refused { ignore(@linear.lu(a)) }) {
    Some(@linear.Refused::Singular(at~)) => assert_eq(at, 1)
    _ => abort("expected Singular")
  }
  let wide = @linear.Mat::of([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
  match refused(fn() raise @linear.Refused { ignore(@linear.lu(wide)) }) {
    Some(@linear.Refused::Dimension(got~, want~)) => {
      assert_eq(got, 3)
      assert_eq(want, 2)
    }
    _ => abort("expected Dimension")
  }
}

///|
test "eigh of a symmetric matrix" {
  let a = @linear.Mat::of([[2.0, 1.0], [1.0, 2.0]])
  let f = @linear.eigh(a)
  assert_eq(
    f.values.approx(@linear.Vec::of([1.0, 3.0]), tolerance=1.0e-12),
    true,
  )
  // A v = lambda v for each column.
  for i in 0..<2 {
    let v = f.vectors.col(i)
    let lambda = @linear.Mat::diagonal(f.values).get(i, i)
    assert_eq(a.apply(v).approx(v.scale(lambda), tolerance=1.0e-12), true)
  }
  assert_eq(
    f.vectors
    .transpose()
    .mul(f.vectors)
    .approx(@linear.Mat::identity(2), tolerance=1.0e-12),
    true,
  )
}

///|
test "eigh of a diagonal matrix keeps the order" {
  let a = @linear.Mat::diagonal(@linear.Vec::of([3.0, 1.0, 2.0]))
  let f = @linear.eigh(a)
  assert_eq(f.values.to_array(), [1.0, 2.0, 3.0])
}

///|
test "expm of a rotation generator is a rotation" {
  let a = @linear.Mat::of([[0.0, -1.0], [1.0, 0.0]])
  let e = @linear.expm(a)
  let c = 0.5403023058681398
  let s = 0.8414709848078965
  assert_eq(
    e.approx(@linear.Mat::of([[c, -s], [s, c]]), tolerance=1.0e-12),
    true,
  )
  assert_eq(
    (@linear.expm(@linear.Mat::identity(2)).get(0, 0) - 2.718281828459045).abs() <
    1.0e-13,
    true,
  )
}

///|
test "norm of a matrix is the Frobenius norm" {
  assert_eq(@linear.norm(@linear.Mat::identity(3)), 3.0.sqrt())
}
