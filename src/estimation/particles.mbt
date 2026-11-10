// The particle filter: a cloud of hypotheses instead of a Gaussian. It is the
// estimator that survives a measurement model the other two cannot express, and
// it is the one that needs a seeded generator to be reproducible.

///|
/// A particle filter over any plant that implements the model contract.
///
/// The particles are drawn around the prior at construction and then carried by
/// the model itself, so the filter never assumes the state distribution stays
/// Gaussian. Resampling is systematic and only runs when the effective sample
/// size falls below half the cloud, which keeps the diversity a plain
/// resample-every-step filter loses.
pub(all) struct ParticleFilter {
  model : &@contract.Model
  process : @linear.Mat
  measurement : @linear.Mat
  mut particles : Array[@linear.Vec]
  mut weights : Array[Double]
  rng : &@rand_contract.Rng
}

///|
/// A particle filter with `samples` particles drawn around `state`.
pub fn ParticleFilter::new(
  model~ : &@contract.Model,
  process~ : @linear.Mat,
  measurement~ : @linear.Mat,
  state~ : @linear.Vec,
  covariance~ : @linear.Mat,
  rng~ : &@rand_contract.Rng,
  samples~ : Int32 = 1000,
) -> ParticleFilter raise @linear.Refused {
  let n = state.len()
  if samples < 2 {
    raise @linear.Refused::Range(got=samples.to_double(), low=2.0, high=1.0e9)
  }
  if covariance.rows != n || covariance.cols != n {
    raise @linear.Refused::Dimension(got=covariance.rows, want=n)
  }
  let jitter = @linear.Mat::identity(n).scale(1.0e-12)
  let factor = @linear.cholesky(covariance.add(jitter))
  let particles = Array::make(samples, state)
  let weights = Array::make(samples, 1.0 / samples.to_double())
  for i in 0..<samples {
    let sample = @linear.Vec::zeros(n)
    // `state + factor * z` with one standard normal per column, which spreads the
    // cloud over the covariance rather than over its diagonal.
    let noise = @linear.Vec::zeros(n)
    for c in 0..<n {
      noise.set(c, @rand.gauss(rng))
    }
    for r in 0..<n {
      let mut value = state.get(r)
      for c in 0..<n {
        value = value + factor.get(r, c) * noise.get(c)
      }
      sample.set(r, value)
    }
    particles[i] = sample
  }
  { model, process, measurement, particles, weights, rng, }
}

///|
/// The particles, for a caller that plots the cloud.
pub fn ParticleFilter::particles(self : ParticleFilter) -> Array[@linear.Vec] {
  self.particles
}

///|
/// The weight of each particle.
pub fn ParticleFilter::weights(self : ParticleFilter) -> Array[Double] {
  self.weights
}

///|
pub impl @contract.Estimator for ParticleFilter with fn predict(self, u, dt) {
  let factor = matrix_sqrt(self.process)
  let n = factor.rows
  for i in 0..<self.particles.length() {
    let moved = self.model.step(self.particles[i], u, dt)
    for r in 0..<n {
      let mut noise = 0.0
      for c in 0..<n {
        noise = noise + factor.get(r, c) * @rand.gauss(self.rng)
      }
      moved.set(r, moved.get(r) + noise)
    }
    self.particles[i] = moved
  }
}

///|
pub impl @contract.Estimator for ParticleFilter with fn correct(self, z) {
  let count = self.particles.length()
  let mut total = 0.0
  for i in 0..<count {
    let predicted = self.model.observe(self.particles[i])
    let innovation = z.sub(predicted)
    let likelihood = gaussian_likelihood(self.measurement, innovation)
    let weight = self.weights[i] * likelihood
    self.weights[i] = weight
    total = total + weight
  }
  if total <= 0.0 || !total.is_finite() {
    // A measurement no particle can explain: fall back to a uniform cloud
    // rather than dividing by zero and losing every hypothesis.
    for i in 0..<count {
      self.weights[i] = 1.0 / count.to_double()
    }
    return
  }
  for i in 0..<count {
    self.weights[i] = self.weights[i] / total
  }
  let mut squared = 0.0
  for i in 0..<count {
    squared = squared + self.weights[i] * self.weights[i]
  }
  let effective = 1.0 / squared
  if effective < count.to_double() / 2.0 {
    self.resample()
  }
}

///|
/// The multivariate Gaussian density of `innovation` under the measurement
/// covariance, up to the normalising constant the weights divide out.
fn gaussian_likelihood(
  covariance : @linear.Mat,
  innovation : @linear.Vec,
) -> Double {
  let m = innovation.len()
  let column = @linear.Mat::zeros(m, 1)
  for i in 0..<m {
    column.set(i, 0, innovation.get(i))
  }
  let exponent = try {
    let solved = @linear.solve(covariance, column).col(0)
    -0.5 * innovation.dot(solved)
  } catch {
    _ => -0.5 * innovation.dot(innovation)
  }
  if exponent < -700.0 {
    0.0
  } else {
    @math.exp(exponent)
  }
}

///|
/// Systematic resampling: one uniform draw, then even steps through the
/// cumulative weights.
fn ParticleFilter::resample(self : ParticleFilter) -> Unit {
  let count = self.particles.length()
  let spacing = 1.0 / count.to_double()
  let first = @rand.random(self.rng) * spacing
  let resampled = Array::make(count, self.particles[0])
  let mut cursor = 0
  let mut cumulative = self.weights[0]
  for i in 0..<count {
    let target = first + spacing * i.to_double()
    while target > cumulative && cursor < count - 1 {
      cursor = cursor + 1
      cumulative = cumulative + self.weights[cursor]
    }
    resampled[i] = self.particles[cursor].clone()
  }
  for i in 0..<count {
    self.particles[i] = resampled[i]
    self.weights[i] = spacing
  }
}

///|
pub impl @contract.Estimator for ParticleFilter with fn state(self) {
  let mut mean = @linear.Vec::zeros(self.particles[0].len())
  for i in 0..<self.particles.length() {
    mean = mean.axpy(self.weights[i], self.particles[i])
  }
  mean
}

///|
pub impl @contract.Estimator for ParticleFilter with fn covariance(self) {
  let mean = self.state()
  let n = mean.len()
  let out = @linear.Mat::zeros(n, n)
  for i in 0..<self.particles.length() {
    let deviation = self.particles[i].sub(mean)
    for r in 0..<n {
      for c in 0..<n {
        out.add_to(r, c, deviation.get(r) * deviation.get(c) * self.weights[i])
      }
    }
  }
  out
}

///|
pub extend ParticleFilter with @contract.Estimator::{
  predict,
  correct,
  state,
  covariance,
}

///|
impl @debug.Debug for ParticleFilter with fn to_repr(self) {
  @debug.Repr::literal(
    "ParticleFilter(" + self.particles.length().to_string() + " particles)",
  )
}

///|
pub extend ParticleFilter with @debug.Debug::{to_repr}
