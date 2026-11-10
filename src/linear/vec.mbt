///|
/// A dense vector of `Double`.
///
/// The storage is a fixed array, so a vector is cheap to pass around and a
/// `set` writes through to it. Every arithmetic method returns a new vector;
/// none of them mutate the receiver.
#derive(Debug)
pub(all) struct Vec {
  data : FixedArray[Double]
}

///|
/// A vector of `len` zeros.
pub fn Vec::zeros(len : Int32) -> Vec {
  { data: FixedArray::make(len, 0.0), }
}

///|
/// A vector of `len` copies of `value`.
pub fn Vec::filled(len : Int32, value : Double) -> Vec {
  { data: FixedArray::make(len, value), }
}

///|
/// A vector holding `values`.
pub fn Vec::of(values : ArrayView[Double]) -> Vec {
  let data = FixedArray::make(values.length(), 0.0)
  for i, v in values {
    data[i] = v
  }
  { data, }
}

///|
/// A vector that takes over `data` without copying it.
pub fn Vec::from_fixed(data : FixedArray[Double]) -> Vec {
  { data, }
}

///|
/// A vector of `len` samples evenly spaced from `start` to `stop` inclusive. A
/// single sample is `start`.
pub fn Vec::linspace(start : Double, stop : Double, len : Int32) -> Vec {
  let data = FixedArray::make(len, 0.0)
  if len == 1 {
    data[0] = start
  } else if len > 1 {
    let step = (stop - start) / (len - 1).to_double()
    for i in 0..<len {
      data[i] = start + step * i.to_double()
    }
  }
  { data, }
}

///|
/// The number of entries.
pub fn Vec::len(self : Vec) -> Int32 {
  self.data.length()
}

///|
/// Whether the vector has no entries.
pub fn Vec::is_empty(self : Vec) -> Bool {
  self.data.length() == 0
}

///|
/// The entry at `i`. An index outside the vector aborts: it is a programming
/// error, not input data.
pub fn Vec::get(self : Vec, i : Int32) -> Double {
  self.data[i]
}

///|
/// The entry at `i`, or `None` when `i` is outside the vector.
pub fn Vec::at(self : Vec, i : Int32) -> Double? {
  if i >= 0 && i < self.data.length() {
    Some(self.data[i])
  } else {
    None
  }
}

///|
/// Overwrite the entry at `i`.
pub fn Vec::set(self : Vec, i : Int32, value : Double) -> Unit {
  self.data[i] = value
}

///|
/// An independent copy.
pub fn Vec::clone(self : Vec) -> Vec {
  let data = FixedArray::make(self.data.length(), 0.0)
  for i in 0..<self.data.length() {
    data[i] = self.data[i]
  }
  { data, }
}

///|
/// The entries copied into a growable array.
pub fn Vec::to_array(self : Vec) -> Array[Double] {
  let out = Array::make(self.data.length(), 0.0)
  for i in 0..<self.data.length() {
    out[i] = self.data[i]
  }
  out
}

///|
/// The elementwise sum. The two vectors must have the same length.
pub fn Vec::add(self : Vec, other : Vec) -> Vec {
  let n = self.data.length()
  if other.data.length() != n {
    abort("Vec::add: length mismatch")
  }
  let data = FixedArray::make(n, 0.0)
  for i in 0..<n {
    data[i] = self.data[i] + other.data[i]
  }
  { data, }
}

///|
/// The elementwise difference. The two vectors must have the same length.
pub fn Vec::sub(self : Vec, other : Vec) -> Vec {
  let n = self.data.length()
  if other.data.length() != n {
    abort("Vec::sub: length mismatch")
  }
  let data = FixedArray::make(n, 0.0)
  for i in 0..<n {
    data[i] = self.data[i] - other.data[i]
  }
  { data, }
}

///|
/// The elementwise product. The two vectors must have the same length.
pub fn Vec::hadamard(self : Vec, other : Vec) -> Vec {
  let n = self.data.length()
  if other.data.length() != n {
    abort("Vec::hadamard: length mismatch")
  }
  let data = FixedArray::make(n, 0.0)
  for i in 0..<n {
    data[i] = self.data[i] * other.data[i]
  }
  { data, }
}

///|
/// Every entry multiplied by `factor`.
pub fn Vec::scale(self : Vec, factor : Double) -> Vec {
  let n = self.data.length()
  let data = FixedArray::make(n, 0.0)
  for i in 0..<n {
    data[i] = self.data[i] * factor
  }
  { data, }
}

///|
/// `self + factor * other`, the operation a matrix-vector product accumulates
/// with.
pub fn Vec::axpy(self : Vec, factor : Double, other : Vec) -> Vec {
  let n = self.data.length()
  if other.data.length() != n {
    abort("Vec::axpy: length mismatch")
  }
  let data = FixedArray::make(n, 0.0)
  for i in 0..<n {
    data[i] = self.data[i] + factor * other.data[i]
  }
  { data, }
}

///|
/// Every entry negated.
pub fn Vec::negate(self : Vec) -> Vec {
  self.scale(-1.0)
}

///|
/// The inner product. The two vectors must have the same length.
pub fn Vec::dot(self : Vec, other : Vec) -> Double {
  let n = self.data.length()
  if other.data.length() != n {
    abort("Vec::dot: length mismatch")
  }
  let mut sum = 0.0
  for i in 0..<n {
    sum += self.data[i] * other.data[i]
  }
  sum
}

///|
/// The sum of the squared entries.
pub fn Vec::norm_squared(self : Vec) -> Double {
  self.dot(self)
}

///|
/// The Euclidean length.
pub fn Vec::norm(self : Vec) -> Double {
  self.dot(self).sqrt()
}

///|
/// The unit vector in the same direction; a zero vector is returned as it is.
pub fn Vec::normalized(self : Vec) -> Vec {
  let n = self.norm()
  if n == 0.0 {
    self.clone()
  } else {
    self.scale(1.0 / n)
  }
}

///|
/// The sum of the entries.
pub fn Vec::sum(self : Vec) -> Double {
  let mut total = 0.0
  for i in 0..<self.data.length() {
    total += self.data[i]
  }
  total
}

///|
/// The largest absolute entry, or zero for an empty vector.
pub fn Vec::max_abs(self : Vec) -> Double {
  let mut best = 0.0
  for i in 0..<self.data.length() {
    let v = self.data[i].abs()
    if v > best {
      best = v
    }
  }
  best
}

///|
/// Whether every entry is finite.
pub fn Vec::is_finite(self : Vec) -> Bool {
  for i in 0..<self.data.length() {
    if !self.data[i].is_finite() {
      return false
    }
  }
  true
}

///|
/// Whether `self` and `other` agree entry by entry within `tolerance`.
pub fn Vec::approx(
  self : Vec,
  other : Vec,
  tolerance~ : Double = 1.0e-9,
) -> Bool {
  let n = self.data.length()
  if other.data.length() != n {
    return false
  }
  for i in 0..<n {
    if (self.data[i] - other.data[i]).abs() > tolerance {
      return false
    }
  }
  true
}

///|
/// The entries in `[ a, b, c ]` form, for output and test expectations.
pub fn Vec::to_string(self : Vec) -> String {
  let parts = Array::make(self.data.length(), "")
  for i in 0..<self.data.length() {
    parts[i] = self.data[i].to_string()
  }
  "[ " + parts.join(", ") + " ]"
}

///|
pub extend Vec with @debug.Debug::{to_repr}
