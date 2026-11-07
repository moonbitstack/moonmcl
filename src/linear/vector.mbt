// The storage is the canonical row-major form of a column, so `row` / `col` on
// a one-element matrix share it.

///|
/// A column vector of `Double`, backed by one flat array. The arithmetic
/// methods return new vectors; `set` is the only writer, and `clone` is what
/// the filters use to keep a history of estimates.
#derive(@debug.Debug)
pub struct Vector {
  // The elements, in order.
  data : Array[Double]
}

///|
pub extend Vector with @debug.Debug::{to_repr}

///|
/// A zero vector of length `n`.
pub fn Vector::zeros(n : Int32) -> Vector {
  { data: Array::makei(n, _ => 0.0), }
}

///|
/// A vector wrapping an existing array of elements.
pub fn Vector::of(data : Array[Double]) -> Vector {
  { data, }
}

///|
/// The `x` coordinate, or 0 when absent.
pub fn Vector::x(self : Vector) -> Double {
  self.get(0).unwrap_or(0.0)
}

///|
/// The `y` coordinate, or 0 when absent.
pub fn Vector::y(self : Vector) -> Double {
  self.get(1).unwrap_or(0.0)
}

///|
/// The `z` coordinate, or 0 when absent.
pub fn Vector::z(self : Vector) -> Double {
  self.get(2).unwrap_or(0.0)
}

///|
/// The number of elements.
pub fn Vector::dim(self : Vector) -> Int32 {
  self.data.length()
}

///|
/// The element at `i` as a `Some`, or `None` when out of range, rather than
/// aborting; index arithmetic on a vector is a common off-by-one source.
pub fn Vector::get(self : Vector, i : Int32) -> Double? {
  if 0 <= i && i < self.data.length() {
    Some(self.data[i])
  } else {
    None
  }
}

///|
/// Overwrite the element at `i`.
pub fn Vector::set(self : Vector, i : Int32, v : Double) -> Unit {
  self.data[i] = v
}

///|
/// The elementwise sum. Both vectors must have the same length.
pub fn Vector::add(self : Vector, other : Vector) -> Vector {
  let out = { data: Array(capacity=self.data.length()), }
  for i in 0..<self.data.length() {
    out.data.push(self.data[i] + other.data[i])
  }
  out
}

///|
/// The elementwise difference. Both vectors must have the same length.
pub fn Vector::sub(self : Vector, other : Vector) -> Vector {
  let out = { data: Array(capacity=self.data.length()), }
  for i in 0..<self.data.length() {
    out.data.push(self.data[i] - other.data[i])
  }
  out
}

///|
/// Every element multiplied by `s`.
pub fn Vector::scale(self : Vector, s : Double) -> Vector {
  { data: [ for v in self.data.iter() => v * s ], }
}

///|
/// The inner (dot) product; both vectors must have the same length.
pub fn Vector::dot(self : Vector, other : Vector) -> Double {
  let mut sum = 0.0
  for i in 0..<self.data.length() {
    sum += self.data[i] * other.data[i]
  }
  sum
}

///|
/// The Euclidean (2-)norm.
pub fn Vector::norm(self : Vector) -> Double {
  self.dot(self).sqrt()
}

///|
/// A unit vector in the same direction; a zero vector is returned unchanged.
pub fn Vector::normalized(self : Vector) -> Vector {
  let n = self.norm()
  if n == 0.0 {
    self
  } else {
    self.scale(1.0 / n)
  }
}

///|
/// An independent copy of the data, safe to mutate on its own.
pub fn Vector::clone(self : Vector) -> Vector {
  { data: self.data.copy(), }
}

///|
/// The elements copied into a plain array.
pub fn Vector::to_array(self : Vector) -> Array[Double] {
  self.data.copy()
}

///|
/// The elements in `[ a, b, c ]` form, for printing and test output.
pub fn Vector::to_string(self : Vector) -> String {
  let parts = [ for v in self.data.iter() => v.to_string() ]
  "[ " + parts.join(", ") + " ]"
}
