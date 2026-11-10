///|
/// The step the numerical Jacobians use, in the units of the state.
///
/// A central difference with this step reaches about seven correct digits for a
/// state of order one, which is the accuracy a filter or a planner needs and
/// far cheaper than a symbolic derivative. An implementation that knows its
/// Jacobian should override the default method instead.
pub let jacobian_step : Double = 1.0e-6

///|
/// A plant: how the state moves under an input, and what a sensor sees.
///
/// `step` and `observe` are the model. `step_jacobian` and `observe_jacobian`
/// have default implementations built on them by central differences, so a
/// plant that gives only the two functions works with the extended Kalman
/// filter, the unscented Kalman filter and the particle filter as it is.
pub(open) trait Model {
  ///|
  /// The state after `dt` seconds from `x` under the held input `u`.
  fn step(Self, x : @linear.Vec, u : @linear.Vec, dt : Double) -> @linear.Vec

  ///|
  /// The measurement a sensor produces for the state `x`.
  fn observe(Self, x : @linear.Vec) -> @linear.Vec

  ///|
  /// `d step / d x`, an `n x n` matrix for an `n`-state.
  fn step_jacobian(Self, x : @linear.Vec, u : @linear.Vec, dt : Double) -> @linear.Mat = _

  ///|
  /// `d observe / d x`, an `m x n` matrix for an `n`-state and an `m`-output.
  fn observe_jacobian(Self, x : @linear.Vec) -> @linear.Mat = _
}

///|
/// The default state Jacobian: a central difference of `step`.
impl Model with fn step_jacobian(self, x, u, dt) {
  let n = x.len()
  let jacobian = @linear.Mat::zeros(n, n)
  for j in 0..<n {
    let bump = @linear.Vec::zeros(n)
    bump.set(j, jacobian_step)
    let plus = self.step(x.add(bump), u, dt)
    let minus = self.step(x.sub(bump), u, dt)
    for i in 0..<n {
      jacobian.set(i, j, (plus.get(i) - minus.get(i)) / (2.0 * jacobian_step))
    }
  }
  jacobian
}

///|
/// The default measurement Jacobian: a central difference of `observe`.
impl Model with fn observe_jacobian(self, x) {
  let n = x.len()
  let m = self.observe(x).len()
  let jacobian = @linear.Mat::zeros(m, n)
  for j in 0..<n {
    let bump = @linear.Vec::zeros(n)
    bump.set(j, jacobian_step)
    let plus = self.observe(x.add(bump))
    let minus = self.observe(x.sub(bump))
    for i in 0..<m {
      jacobian.set(i, j, (plus.get(i) - minus.get(i)) / (2.0 * jacobian_step))
    }
  }
  jacobian
}
