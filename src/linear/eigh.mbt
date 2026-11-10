///|
/// The eigenvalues and eigenvectors of a symmetric matrix.
///
/// `values` is ascending and `vectors` has the matching eigenvector in its
/// matching column, so `a * v[:, i] = values[i] * v[:, i]`.
#derive(Debug)
pub(all) struct Eigh {
  values : Vec
  vectors : Mat
}

///|
/// The symmetric eigendecomposition of `a` by cyclic Jacobi rotations.
///
/// The rotations are stable for the covariance-shaped matrices this library
/// hands it, and they need no shift strategy. Only the symmetric part of `a` is
/// used, so a small skew from rounding does not matter. Raises
/// `Refused::Dimension` when `a` is not square and `Refused::Exceeded` when the
/// sweep limit is reached.
pub fn eigh(a : Mat, sweeps~ : Int32 = 64) -> Eigh raise Refused {
  let n = a.rows
  if a.rows != a.cols {
    raise Refused::Dimension(got=a.cols, want=a.rows)
  }
  if n == 0 {
    raise Refused::Empty
  }
  let m = Mat::zeros(n, n)
  for i in 0..<n {
    for j in 0..<n {
      m.set(i, j, 0.5 * (a.get(i, j) + a.get(j, i)))
    }
  }
  let v = Mat::identity(n)
  let mut scale = 0.0
  for i in 0..<n {
    for j in 0..<n {
      scale += m.get(i, j) * m.get(i, j)
    }
  }
  scale = scale.sqrt()
  let tolerance = 1.0e-15 * (if scale == 0.0 { 1.0 } else { scale })
  let mut sweep = 0
  while sweep < sweeps {
    let mut off = 0.0
    for i in 0..<n {
      for j in (i + 1)..<n {
        off += m.get(i, j) * m.get(i, j)
      }
    }
    if off.sqrt() <= tolerance {
      break
    }
    for p in 0..<(n - 1) {
      for q in (p + 1)..<n {
        let apq = m.get(p, q)
        if apq.abs() <= tolerance * 1.0e-3 {
          continue
        }
        let theta = (m.get(q, q) - m.get(p, p)) / (2.0 * apq)
        let t = if theta >= 0.0 {
          1.0 / (theta + (1.0 + theta * theta).sqrt())
        } else {
          -1.0 / (-theta + (1.0 + theta * theta).sqrt())
        }
        let c = 1.0 / (1.0 + t * t).sqrt()
        let s = t * c
        for k in 0..<n {
          let mkp = m.get(k, p)
          let mkq = m.get(k, q)
          m.set(k, p, c * mkp - s * mkq)
          m.set(k, q, s * mkp + c * mkq)
        }
        for k in 0..<n {
          let mpk = m.get(p, k)
          let mqk = m.get(q, k)
          m.set(p, k, c * mpk - s * mqk)
          m.set(q, k, s * mpk + c * mqk)
        }
        for k in 0..<n {
          let vkp = v.get(k, p)
          let vkq = v.get(k, q)
          v.set(k, p, c * vkp - s * vkq)
          v.set(k, q, s * vkp + c * vkq)
        }
      }
    }
    sweep += 1
  }
  if sweep >= sweeps {
    raise Refused::Exceeded(limit=sweeps, got=sweeps)
  }
  let values = Vec::zeros(n)
  for i in 0..<n {
    values.set(i, m.get(i, i))
  }
  let order = Array::make(n, 0)
  for i in 0..<n {
    order[i] = i
  }
  for i in 0..<n {
    let mut best = i
    for j in (i + 1)..<n {
      if values.get(j) < values.get(best) {
        best = j
      }
    }
    if best != i {
      let t = values.get(i)
      values.set(i, values.get(best))
      values.set(best, t)
      let o = order[i]
      order[i] = order[best]
      order[best] = o
    }
  }
  let vectors = Mat::zeros(n, n)
  for j in 0..<n {
    for i in 0..<n {
      vectors.set(i, j, v.get(i, order[j]))
    }
  }
  { values, vectors, }
}

///|
pub extend Eigh with @debug.Debug::{to_repr}
