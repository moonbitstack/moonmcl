///|
/// A mechanism: joint values to a task-space pose, and back.
///
/// Both spaces are plain `Vec`s so that one contract covers a two-wheel base and
/// a six-axis arm; a model that has typed joint values offers them alongside.
pub(open) trait Kinematics {
  ///|
  /// The task-space pose for the joint values `q`.
  fn forward(Self, q : @linear.Vec) -> @linear.Vec

  ///|
  /// The joint values that reach the pose `x`. `seed` starts an iterative
  /// solve; without it the model picks its own starting point.
  fn inverse(Self, x : @linear.Vec, seed? : @linear.Vec) -> @linear.Vec raise @linear.Refused

  ///|
  /// `d forward / d q`, the task-space rows by the joint columns.
  fn jacobian(Self, q : @linear.Vec) -> @linear.Mat = _
}

///|
/// The default Jacobian: a central difference of `forward`, which is what a
/// mechanism without an analytic Jacobian gets.
impl Kinematics with fn jacobian(self, q) {
  let n = q.len()
  let rows = self.forward(q).len()
  let jacobian = @linear.Mat::zeros(rows, n)
  for j in 0..<n {
    let bump = @linear.Vec::zeros(n)
    bump.set(j, jacobian_step)
    let plus = self.forward(q.add(bump))
    let minus = self.forward(q.sub(bump))
    for i in 0..<rows {
      jacobian.set(i, j, (plus.get(i) - minus.get(i)) / (2.0 * jacobian_step))
    }
  }
  jacobian
}
