// Tests for the linear package: vectors, matrix products, the LU solve, the
// inverse, the Kronecker product and the QR factorisation.

///|
test "vector zeros and accessors" {
  let v = @linear.Vector::zeros(3)
  assert_eq(v.dim(), 3)
  assert_eq(v.x(), 0.0)
  assert_eq(v.y(), 0.0)
  assert_eq(v.z(), 0.0)
  v.set(1, 2.5)
  assert_eq(v.y(), 2.5)
  assert_eq(v.get(5), None)
}

///|
test "vector dot norm normalized" {
  let a = @linear.Vector::of([3.0, 4.0])
  let b = @linear.Vector::of([1.0, 2.0])
  assert_eq(a.dot(b), 11.0)
  assert_eq(a.norm(), 5.0)
  let n = a.normalized()
  assert_eq((n.norm() - 1.0).abs() < 1.0e-9, true)
  assert_eq((n.x() - 0.6).abs() < 1.0e-9, true)
  assert_eq((n.y() - 0.8).abs() < 1.0e-9, true)
}

///|
test "vector add sub scale" {
  let a = @linear.Vector::of([1.0, 2.0, 3.0])
  let b = @linear.Vector::of([4.0, 5.0, 6.0])
  let s = a.add(b)
  assert_eq(s.to_array(), [5.0, 7.0, 9.0])
  let d = a.sub(b)
  assert_eq(d.to_array(), [-3.0, -3.0, -3.0])
  let k = a.scale(2.0)
  assert_eq(k.to_array(), [2.0, 4.0, 6.0])
}

///|
test "matrix identity and transpose" {
  let i = @linear.Matrix::identity(3)
  assert_eq(i.at(0, 0), Some(1.0))
  assert_eq(i.at(0, 1), Some(0.0))
  assert_eq(i.at(2, 2), Some(1.0))
  let m = @linear.Matrix::of([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
  let t = m.transpose()
  assert_eq(t.rows, 2)
  assert_eq(t.cols, 3)
  assert_eq(t.at(0, 0), Some(1.0))
  assert_eq(t.at(0, 1), Some(3.0))
  assert_eq(t.at(0, 2), Some(5.0))
  assert_eq(t.at(1, 2), Some(6.0))
}

///|
test "matrix mat_vec and mat_mat" {
  let a = @linear.Matrix::of([[1.0, 2.0], [3.0, 4.0]])
  let v = @linear.Vector::of([1.0, 1.0])
  let p = a.mat_vec(v)
  assert_eq(p.to_array(), [3.0, 7.0])
  let b = @linear.Matrix::of([[0.0, 1.0], [1.0, 0.0]])
  let c = a.mat_mat(b)
  assert_eq(c.at(0, 0), Some(2.0))
  assert_eq(c.at(0, 1), Some(1.0))
  assert_eq(c.at(1, 0), Some(4.0))
  assert_eq(c.at(1, 1), Some(3.0))
}

///|
test "matrix solve recovers a known solution" {
  // A = [[4, 1], [1, 3]], solution x = (2, 1): A x = (9, 5).
  let a = @linear.Matrix::of([[4.0, 1.0], [1.0, 3.0]])
  let b = @linear.Vector::of([9.0, 5.0])
  let x = a.solve(b)
  assert_eq((x.x() - 2.0).abs() < 1.0e-9, true)
  assert_eq((x.y() - 1.0).abs() < 1.0e-9, true)
}

///|
test "matrix solve of a 3x3 system" {
  // A x = b with x = (1, 2, 3).
  let a = @linear.Matrix::of([[2.0, 0.0, 0.0], [0.0, 3.0, 0.0], [0.0, 0.0, 4.0]])
  let b = @linear.Vector::of([2.0, 6.0, 12.0])
  let x = a.solve(b)
  assert_eq((x.x() - 1.0).abs() < 1.0e-9, true)
  assert_eq((x.y() - 2.0).abs() < 1.0e-9, true)
  assert_eq((x.z() - 3.0).abs() < 1.0e-9, true)
  // A coupled 3x3 with a known solution.
  let a2 = @linear.Matrix::of([
    [3.0, 2.0, -1.0],
    [2.0, -2.0, 4.0],
    [-1.0, 0.5, -1.0],
  ])
  let b2 = @linear.Vector::of([1.0, -2.0, 0.0])
  let x2 = a2.solve(b2)
  let r2 = a2.mat_vec(x2)
  assert_eq((r2.x() - 1.0).abs() < 1.0e-8, true)
  assert_eq((r2.y() + 2.0).abs() < 1.0e-8, true)
  assert_eq(r2.z().abs() < 1.0e-8, true)
}

///|
test "matrix inverse times original is identity" {
  let a = @linear.Matrix::of([[4.0, 7.0], [2.0, 6.0]])
  let ai = a.inv()
  let prod = a.mat_mat(ai)
  assert_eq((prod.at(0, 0).unwrap() - 1.0).abs() < 1.0e-9, true)
  assert_eq(prod.at(0, 1).unwrap().abs() < 1.0e-9, true)
  assert_eq(prod.at(1, 0).unwrap().abs() < 1.0e-9, true)
  assert_eq((prod.at(1, 1).unwrap() - 1.0).abs() < 1.0e-9, true)
}

///|
test "kron product of two 2x2" {
  let a = @linear.Matrix::of([[1.0, 2.0], [3.0, 4.0]])
  let b = @linear.Matrix::identity(2)
  let k = a.kron(b)
  assert_eq(k.rows, 4)
  assert_eq(k.cols, 4)
  assert_eq(k.at(0, 0), Some(1.0))
  assert_eq(k.at(0, 1), Some(0.0))
  assert_eq(k.at(0, 2), Some(2.0))
  assert_eq(k.at(2, 0), Some(3.0))
  assert_eq(k.at(3, 3), Some(4.0))
}

///|
test "qr factorisation has orthonormal q" {
  let a = @linear.Matrix::of([[1.0, 2.0], [0.0, 1.0], [1.0, 0.0]])
  let (q, _r) = a.qr()
  let qt = q.transpose()
  let qqt = qt.mat_mat(q)
  assert_eq((qqt.at(0, 0).unwrap() - 1.0).abs() < 1.0e-9, true)
  assert_eq(qqt.at(0, 1).unwrap().abs() < 1.0e-9, true)
  assert_eq(qqt.at(1, 0).unwrap().abs() < 1.0e-9, true)
  assert_eq((qqt.at(1, 1).unwrap() - 1.0).abs() < 1.0e-9, true)
}
