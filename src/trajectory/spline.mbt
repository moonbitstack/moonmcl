// Two ways through waypoints: a natural cubic spline, which passes through them,
// and a clamped B-spline, which is pulled towards them without touching them.
// The first is for a path that has to be followed exactly, the second for one
// that only has to be smooth.

///|
/// A natural cubic spline through time-stamped positions, continuous in
/// velocity and acceleration, with zero curvature at both ends.
#derive(Debug)
pub(all) struct CubicSpline {
  times : Array[Double]
  positions : Array[Double]
  curvatures : Array[Double]
}

///|
/// A natural cubic spline through `(time, position)` knots, which must be
/// ordered in time.
pub fn cubic_spline(
  knots~ : ArrayView[(Double, Double)],
) -> CubicSpline raise @linear.Refused {
  let count = knots.length()
  if count < 2 {
    raise @linear.Refused::Dimension(got=count, want=2)
  }
  let times = Array::make(count, 0.0)
  let positions = Array::make(count, 0.0)
  for i in 0..<count {
    times[i] = knots[i].0
    positions[i] = knots[i].1
  }
  for i in 1..<count {
    if times[i] <= times[i - 1] {
      raise @linear.Refused::Range(
        got=times[i],
        low=times[i - 1] + 1.0e-12,
        high=1.0e12,
      )
    }
  }
  // The second derivatives at the knots solve a tridiagonal system with zero
  // curvature at both ends; the Thomas sweep is enough for it.
  let curvatures = Array::make(count, 0.0)
  if count > 2 {
    let inner = count - 2
    let diagonal = Array::make(inner, 0.0)
    let upper = Array::make(inner, 0.0)
    let lower = Array::make(inner, 0.0)
    let right = Array::make(inner, 0.0)
    for i in 0..<inner {
      let k = i + 1
      let previous = times[k] - times[k - 1]
      let next = times[k + 1] - times[k]
      diagonal[i] = 2.0 * (previous + next)
      if i > 0 {
        lower[i] = previous
      }
      if i < inner - 1 {
        upper[i] = next
      }
      let slope_next = (positions[k + 1] - positions[k]) / next
      let slope_previous = (positions[k] - positions[k - 1]) / previous
      right[i] = 6.0 * (slope_next - slope_previous)
    }
    // Forward sweep.
    for i in 1..<inner {
      let factor = lower[i] / diagonal[i - 1]
      diagonal[i] = diagonal[i] - factor * upper[i - 1]
      right[i] = right[i] - factor * right[i - 1]
    }
    curvatures[inner] = right[inner - 1] / diagonal[inner - 1]
    for i in (inner - 2)>=..0 {
      curvatures[i + 1] = (right[i] - upper[i] * curvatures[i + 2]) /
        diagonal[i]
    }
  }
  { times, positions, curvatures, }
}

///|
/// The interval index for `t`, clamped to the ends.
fn CubicSpline::segment(self : CubicSpline, t : Double) -> Int32 {
  let count = self.times.length()
  if t <= self.times[0] {
    return 0
  }
  if t >= self.times[count - 1] {
    return count - 2
  }
  let mut low = 0
  let mut high = count - 1
  while high - low > 1 {
    let middle = (low + high) / 2
    if self.times[middle] <= t {
      low = middle
    } else {
      high = middle
    }
  }
  low
}

///|
pub impl @contract.Trajectory for CubicSpline with fn duration(self) {
  self.times[self.times.length() - 1]
}

///|
pub impl @contract.Trajectory for CubicSpline with fn sample(self, t) {
  let index = self.segment(t)
  let left = self.times[index]
  let width = self.times[index + 1] - left
  let offset = t - left
  let a = self.curvatures[index]
  let b = self.curvatures[index + 1]
  let slope = (self.positions[index + 1] - self.positions[index]) / width -
    width * (2.0 * a + b) / 6.0
  let position = self.positions[index] +
    slope * offset +
    0.5 * a * offset * offset +
    (b - a) / (6.0 * width) * offset * offset * offset
  let velocity = slope + a * offset + 0.5 * (b - a) / width * offset * offset
  let acceleration = a + (b - a) / width * offset
  let jerk = (b - a) / width
  @contract.Sample::new(position~, velocity~, acceleration~, jerk~)
}

///|
pub extend CubicSpline with @contract.Trajectory::{duration, sample}

///|
pub extend CubicSpline with @debug.Debug::{to_repr}

///|
/// A clamped uniform B-spline over a control polygon.
///
/// The curve is pulled towards the control points without passing through them,
/// which is what makes it the profile for a path that only has to be smooth. The
/// first and last control points are interpolated, so it starts and ends where
/// the polygon does.
#derive(Debug)
pub(all) struct BSpline {
  degree : Int32
  control : Array[Double]
  knots : Array[Double]
  duration : Double
}

///|
/// A clamped B-spline of `degree` over `control`, running for `duration`
/// seconds.
pub fn b_spline(
  control~ : ArrayView[Double],
  degree~ : Int32 = 3,
  duration~ : Double = 1.0,
) -> BSpline raise @linear.Refused {
  let count = control.length()
  if degree < 1 {
    raise @linear.Refused::Range(got=degree.to_double(), low=1.0, high=16.0)
  }
  if count < degree + 1 {
    raise @linear.Refused::Dimension(got=count, want=degree + 1)
  }
  if duration <= 0.0 {
    raise @linear.Refused::Range(got=duration, low=1.0e-12, high=1.0e12)
  }
  let points = Array::make(count, 0.0)
  for i in 0..<count {
    points[i] = control[i]
  }
  // A clamped uniform knot vector: `degree + 1` zeros, `degree + 1` ones, and an
  // even interior.
  let inner = count - degree - 1
  let knots = Array::make(count + degree + 1, 0.0)
  for i in 0..<inner {
    knots[degree + 1 + i] = (i + 1).to_double() / (inner + 1).to_double()
  }
  for i in count..<(count + degree + 1) {
    knots[i] = 1.0
  }
  { degree, control: points, knots, duration, }
}

///|
/// The value of a B-spline at the parameter `u` in `[0, 1]`, by de Boor's
/// algorithm.
fn de_boor(
  control : Array[Double],
  knots : Array[Double],
  degree : Int32,
  u : Double,
) -> Double {
  let last = control.length() - 1
  let mut span = degree
  while span < last && u >= knots[span + 1] {
    span = span + 1
  }
  let work = Array::make(degree + 1, 0.0)
  for j in 0..<=degree {
    work[j] = control[span - degree + j]
  }
  for r in 1..<=degree {
    for j in degree>=..r {
      let i = span - degree + j
      let width = knots[i + degree - r + 1] - knots[i]
      let alpha = if width > 0.0 { (u - knots[i]) / width } else { 0.0 }
      work[j] = (1.0 - alpha) * work[j - 1] + alpha * work[j]
    }
  }
  work[degree]
}

///|
/// The control polygon of the derivative, one degree lower.
fn derivative_control(
  control : Array[Double],
  knots : Array[Double],
  degree : Int32,
) -> Array[Double] {
  let count = control.length() - 1
  let out = Array::make(count, 0.0)
  for i in 0..<count {
    let width = knots[i + degree + 1] - knots[i + 1]
    out[i] = if width > 0.0 {
      degree.to_double() * (control[i + 1] - control[i]) / width
    } else {
      0.0
    }
  }
  out
}

///|
pub impl @contract.Trajectory for BSpline with fn duration(self) {
  self.duration
}

///|
pub impl @contract.Trajectory for BSpline with fn sample(self, t) {
  let u = if t <= 0.0 {
    0.0
  } else if t >= self.duration {
    1.0
  } else {
    t / self.duration
  }
  let first = derivative_control(self.control, self.knots, self.degree)
  let position = de_boor(self.control, self.knots, self.degree, u)
  let velocity = if self.degree >= 1 {
    de_boor(first, self.knots, self.degree - 1, u) / self.duration
  } else {
    0.0
  }
  let acceleration = if self.degree >= 2 {
    let second = derivative_control(first, self.knots, self.degree - 1)
    de_boor(second, self.knots, self.degree - 2, u) /
    (self.duration * self.duration)
  } else {
    0.0
  }
  let jerk = if self.degree >= 3 {
    let second = derivative_control(first, self.knots, self.degree - 1)
    let third = derivative_control(second, self.knots, self.degree - 2)
    de_boor(third, self.knots, self.degree - 3, u) /
    (self.duration * self.duration * self.duration)
  } else {
    0.0
  }
  @contract.Sample::new(position~, velocity~, acceleration~, jerk~)
}

///|
pub extend BSpline with @contract.Trajectory::{duration, sample}

///|
pub extend BSpline with @debug.Debug::{to_repr}
