///|
/// An economy QR factorisation, `a = q * r`, with `q` of orthonormal columns
/// and `r` upper triangular.
///
/// `q` has the same shape as `a` and `r` is square, which is the form the
/// least-squares step and the UKF's square root need.
#derive(Debug)
pub(all) struct Qr {
  q : Mat
  r : Mat
}

///|
/// Factorise `a` as `q * r` with Householder reflections.
///
/// Raises `Refused::Dimension` when `a` has fewer rows than columns: an economy
/// factorisation cannot represent that shape.
pub fn qr(a : Mat) -> Qr raise Refused {
  let m = a.rows
  let n = a.cols
  if n == 0 || m == 0 {
    raise Refused::Empty
  }
  if m < n {
    raise Refused::Dimension(got=m, want=n)
  }
  let work = Mat::zeros(m, n)
  for i in 0..<m {
    for j in 0..<n {
      work.set(i, j, a.get(i, j))
    }
  }
  let reflectors = Array::make(n, Vec::zeros(0))
  for k in 0..<n {
    let length = m - k
    let x = Vec::zeros(length)
    for i in 0..<length {
      x.set(i, work.get(k + i, k))
    }
    let norm = x.norm()
    if norm == 0.0 {
      reflectors[k] = Vec::zeros(0)
      continue
    }
    let alpha = if x.get(0) >= 0.0 { -norm } else { norm }
    let v = Vec::zeros(length)
    for i in 0..<length {
      v.set(i, x.get(i))
    }
    v.set(0, v.get(0) - alpha)
    let vnorm = v.norm()
    if vnorm == 0.0 {
      reflectors[k] = Vec::zeros(0)
      continue
    }
    for i in 0..<length {
      v.set(i, v.get(i) / vnorm)
    }
    reflectors[k] = v
    for j in k..<n {
      let mut s = 0.0
      for i in 0..<length {
        s += v.get(i) * work.get(k + i, j)
      }
      s = 2.0 * s
      for i in 0..<length {
        work.set(k + i, j, work.get(k + i, j) - s * v.get(i))
      }
    }
  }
  let q = Mat::zeros(m, n)
  for j in 0..<n {
    q.set(j, j, 1.0)
  }
  for k in (n - 1)>=..0 {
    let v = reflectors[k]
    let length = v.len()
    if length == 0 {
      continue
    }
    for j in 0..<n {
      let mut s = 0.0
      for i in 0..<length {
        s += v.get(i) * q.get(k + i, j)
      }
      s = 2.0 * s
      for i in 0..<length {
        q.set(k + i, j, q.get(k + i, j) - s * v.get(i))
      }
    }
  }
  let r = Mat::zeros(n, n)
  for i in 0..<n {
    for j in 0..<n {
      r.set(i, j, work.get(i, j))
    }
  }
  { q, r, }
}

///|
pub extend Qr with @debug.Debug::{to_repr}
