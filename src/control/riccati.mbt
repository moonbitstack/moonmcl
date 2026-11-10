///|
/// The solution of a discrete algebraic Riccati equation: the stabilising gain
/// and the cost-to-go matrix it came with.
#derive(Debug)
pub(all) struct Riccati {
  gain : @linear.Mat
  cost : @linear.Mat
}

///|
/// Solve the discrete algebraic Riccati equation for `(a, b)` at the weights
/// `(q, r)` by iterating the Riccati recursion to a fixed point.
///
/// The recursion is monotone from `p = q`, so it converges to the stabilising
/// solution without a Schur decomposition. A system that cannot be stabilised
/// does not converge, and the iteration says so with `Refused::Exceeded` rather
/// than returning a gain that would drive it further away.
pub fn dare(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  iterations~ : Int32 = 2000,
  tolerance~ : Double = 1.0e-10,
) -> Riccati raise @linear.Refused {
  let n = a.rows
  if a.rows != a.cols {
    raise @linear.Refused::Dimension(got=a.cols, want=a.rows)
  }
  if b.rows != n {
    raise @linear.Refused::Dimension(got=b.rows, want=n)
  }
  if q.rows != n || q.cols != n {
    raise @linear.Refused::Dimension(got=q.rows, want=n)
  }
  if r.rows != b.cols || r.cols != b.cols {
    raise @linear.Refused::Dimension(got=r.rows, want=b.cols)
  }
  let at = a.transpose()
  let mut p = q
  let mut gain = @linear.Mat::zeros(b.cols, n)
  for step in 0..<iterations {
    let btp = b.transpose().mul(p)
    let s = r.add(btp.mul(b))
    let bpa = btp.mul(a)
    let k = @linear.solve(s, bpa)
    let next = q.add(at.mul(p).mul(a)).sub(bpa.transpose().mul(k))
    let mut delta = 0.0
    let mut scale = 1.0
    for i in 0..<n {
      for j in 0..<n {
        let d = (next.get(i, j) - p.get(i, j)).abs()
        if d > delta {
          delta = d
        }
        if next.get(i, j).abs() > scale {
          scale = next.get(i, j).abs()
        }
      }
    }
    p = next
    gain = k
    // Relative to the size of the cost, because an augmented plant with an
    // integrator reaches entries in the thousands and an absolute test would
    // never be met.
    if delta <= tolerance * scale {
      return { gain, cost: p, }
    }
    ignore(step)
  }
  raise @linear.Refused::Exceeded(limit=iterations, got=iterations)
}

///|
pub extend Riccati with @debug.Debug::{to_repr}
