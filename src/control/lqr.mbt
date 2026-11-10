///|
/// A linear-quadratic regulator: the infinite-horizon gain for a discrete plant
/// and quadratic weights.
///
/// The control law is `u = -k (x - reference)`, so a regulator that drives the
/// plant to a setpoint is the same object as the one that drives it to zero.
pub(all) struct Lqr {
  gain : @linear.Mat
}

///|
/// The infinite-horizon LQR for a discrete `(a, b)` pair. An element of `r` that
/// is not positive is refused: it would let the input be free.
pub fn lqr(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  iterations~ : Int32 = 2000,
  tolerance~ : Double = 1.0e-10,
) -> Lqr raise @linear.Refused {
  for i in 0..<r.rows {
    if r.get(i, i) <= 0.0 {
      raise @linear.Refused::Gain(got=r.get(i, i))
    }
  }
  let solution = dare(a~, b~, q~, r~, iterations~, tolerance~)
  { gain: solution.gain, }
}

///|
/// The gain matrix, for a caller that applies it itself.
pub fn Lqr::gain(self : Lqr) -> @linear.Mat {
  self.gain
}

///|
pub impl @contract.Controller for Lqr with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  ignore(dt)
  let error = measurement.sub(reference)
  self.gain.apply(error).scale(-1.0)
}

///|
pub impl @contract.Controller for Lqr with fn reset(self) {
  ignore(self)
}

///|
pub extend Lqr with @contract.Controller::{update, reset}

///|
/// An LQR with an integral of the output error appended to the state.
///
/// The extra state is what removes a constant disturbance's offset: the
/// proportional part of an LQR leaves a standing error when the plant is pushed
/// by a bias, and the integrator integrates that error away. The integral acts on
/// the output `c x`, not on the whole state, because integrating a state the
/// input cannot reach independently would leave the augmented plant
/// unstabilisable.
pub(all) struct IntegralLqr {
  gain : @linear.Mat
  state_dim : Int32
  output : @linear.Mat
  dt : Double
  mut integral : @linear.Vec
}

///|
/// The integral LQR for a discrete `(a, b)` pair with outputs `c`, accumulating
/// the output error at `dt` seconds.
pub fn integral_lqr(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  c~ : @linear.Mat,
  q~ : @linear.Mat,
  r~ : @linear.Mat,
  integral_weight~ : @linear.Mat,
  dt~ : Double,
  iterations~ : Int32 = 2000,
  tolerance~ : Double = 1.0e-10,
) -> IntegralLqr raise @linear.Refused {
  let n = a.rows
  if dt <= 0.0 {
    raise @linear.Refused::Range(got=dt, low=1.0e-12, high=1.0e12)
  }
  if integral_weight.rows != c.rows || integral_weight.cols != c.rows {
    raise @linear.Refused::Dimension(got=integral_weight.rows, want=c.rows)
  }
  if c.cols != n {
    raise @linear.Refused::Dimension(got=c.cols, want=n)
  }
  let p = c.rows
  let zeros_state = @linear.Mat::zeros(n, p)
  let upper_a = a.augment(zeros_state)
  let lower_a = c.scale(dt).augment(@linear.Mat::identity(p))
  let a_aug = upper_a.stack(lower_a)
  let b_aug = b.stack(@linear.Mat::zeros(p, b.cols))
  let q_aug = q
    .augment(@linear.Mat::zeros(n, p))
    .stack(@linear.Mat::zeros(p, n).augment(integral_weight))
  let solution = dare(a=a_aug, b=b_aug, q=q_aug, r~, iterations~, tolerance~)
  {
    gain: solution.gain,
    state_dim: n,
    output: c,
    dt,
    integral: @linear.Vec::zeros(p),
  }
}

///|
/// The gain matrix over `[state; integral]`.
pub fn IntegralLqr::gain(self : IntegralLqr) -> @linear.Mat {
  self.gain
}

///|
pub impl @contract.Controller for IntegralLqr with fn update(
  self,
  reference,
  measurement,
  dt,
) {
  let n = self.state_dim
  let p = self.output.rows
  let error = measurement.sub(reference)
  let projected = self.output.apply(error)
  for i in 0..<p {
    self.integral.set(i, self.integral.get(i) + projected.get(i) * dt)
  }
  let proportional = self.gain.block(
    row=0,
    col=0,
    height=self.gain.rows,
    width=n,
  )
  let integral = self.gain.block(row=0, col=n, height=self.gain.rows, width=p)
  proportional.apply(error).add(integral.apply(self.integral)).scale(-1.0)
}

///|
pub impl @contract.Controller for IntegralLqr with fn reset(self) {
  for i in 0..<self.integral.len() {
    self.integral.set(i, 0.0)
  }
}

///|
pub extend IntegralLqr with @contract.Controller::{update, reset}
