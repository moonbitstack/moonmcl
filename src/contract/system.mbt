// The two ways a plant enters this library: as a linear state-space record or
// as a pair of functions with an integrator. Both implement `Model`, so nothing
// downstream cares which one it was handed.

///|
/// Which rule turns a continuous state matrix into a discrete one.
#derive(Debug)
pub(all) enum Discretize {
  ///
  /// Zero-order hold: the input is held over the period, which is what a
  /// digital controller actually does.
  Zoh
  ///
  /// Forward Euler: `a_d = i + a * dt`. Cheap, and unstable for a period that
  /// is long against the plant's fastest pole.
  ForwardEuler
  ///
  /// Tustin: the trapezoidal rule, which keeps the left-half plane mapped into
  /// the unit disc.
  Tustin
}

///|
/// A linear time-invariant plant, `x' = a x + b u`, `y = c x + d u`.
///
/// With `dt` set the record is already discrete and `step` applies `a x + b u`
/// once; without it the record is continuous and `step` integrates over the
/// period it is given. A discrete record is what the Kalman filter and the
/// model-predictive controller want; a continuous one is what a plant model in
/// physical units looks like.
#derive(Debug)
pub(all) struct StateSpace {
  a : @linear.Mat
  b : @linear.Mat
  c : @linear.Mat
  d : @linear.Mat
  dt : Double?
}

///|
/// A continuous plant.
pub fn StateSpace::continuous(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  c~ : @linear.Mat,
  d~ : @linear.Mat,
) -> StateSpace raise @linear.Refused {
  validate_shapes(a, b, c, d)
  { a, b, c, d, dt: None, }
}

///|
/// A plant already discretised at `dt` seconds.
pub fn StateSpace::discrete(
  a~ : @linear.Mat,
  b~ : @linear.Mat,
  c~ : @linear.Mat,
  d~ : @linear.Mat,
  dt~ : Double,
) -> StateSpace raise @linear.Refused {
  validate_shapes(a, b, c, d)
  if dt <= 0.0 {
    raise @linear.Refused::Range(got=dt, low=1.0e-12, high=1.0e12)
  }
  { a, b, c, d, dt: Some(dt), }
}

///|
fn validate_shapes(
  a : @linear.Mat,
  b : @linear.Mat,
  c : @linear.Mat,
  d : @linear.Mat,
) -> Unit raise @linear.Refused {
  if a.rows != a.cols {
    raise @linear.Refused::Dimension(got=a.cols, want=a.rows)
  }
  if b.rows != a.rows {
    raise @linear.Refused::Dimension(got=b.rows, want=a.rows)
  }
  if c.cols != a.cols {
    raise @linear.Refused::Dimension(got=c.cols, want=a.cols)
  }
  if d.rows != c.rows || d.cols != b.cols {
    raise @linear.Refused::Dimension(got=d.rows, want=c.rows)
  }
}

///|
/// Whether the record is already discrete.
pub fn StateSpace::is_discrete(self : StateSpace) -> Bool {
  self.dt is Some(_)
}

///|
/// The same plant discretised at `dt` seconds.
///
/// A record that is already discrete is returned unchanged when the periods
/// agree and refused when they do not: resampling would need the original
/// continuous plant, which the record no longer carries.
pub fn StateSpace::discretize(
  self : StateSpace,
  dt~ : Double,
  discretize~ : Discretize = Zoh,
) -> StateSpace raise @linear.Refused {
  if dt <= 0.0 {
    raise @linear.Refused::Range(got=dt, low=1.0e-12, high=1.0e12)
  }
  match self.dt {
    Some(existing) =>
      if (existing - dt).abs() <= 1.0e-12 * dt {
        return self
      } else {
        raise @linear.Refused::Range(got=dt, low=existing, high=existing)
      }
    None => ()
  }
  let n = self.a.rows
  let m = self.b.cols
  let (a_d, b_d) = match discretize {
    Zoh => {
      // expm([[a, b], [0, 0]] * dt) keeps the top-left block and the top-right
      // block, which are exactly the zero-order-hold pair.
      let augmented = self.a.augment(self.b).stack(@linear.Mat::zeros(m, n + m))
      let scaled = augmented.scale(dt)
      let e = @linear.expm(scaled)
      (
        e.block(row=0, col=0, height=n, width=n),
        e.block(row=0, col=n, height=n, width=m),
      )
    }
    ForwardEuler =>
      (@linear.Mat::identity(n).add(self.a.scale(dt)), self.b.scale(dt))
    Tustin => {
      let half = dt / 2.0
      let shifted = @linear.Mat::identity(n).sub(self.a.scale(half))
      let leading = @linear.Mat::identity(n).add(self.a.scale(half))
      let a_d = @linear.solve(shifted, leading)
      let b_d = @linear.solve(shifted, self.b.scale(dt))
      (a_d, b_d)
    }
  }
  { a: a_d, b: b_d, c: self.c, d: self.d, dt: Some(dt), }
}

///|
/// One period of the plant.
pub impl Model for StateSpace with fn step(self, x, u, dt) {
  let state = match self.dt {
    Some(_) => self.a.apply(x).add(self.b.apply(u))
    None => {
      let derivative = self.a.apply(x).add(self.b.apply(u))
      x.add(derivative.scale(dt))
    }
  }
  state
}

///|
/// The measurement the plant's outputs give for a state.
pub impl Model for StateSpace with fn observe(self, x) {
  self.c.apply(x)
}

///|
/// How `Dynamics` integrates its derivative.
#derive(Debug)
pub(all) enum Integrator {
  ///
  /// One evaluation at the start of the period.
  Euler
  ///
  /// Two evaluations, at the start and the middle.
  Midpoint
  ///
  /// Four evaluations, the classical fourth-order rule.
  Rk4
}

///|
/// A nonlinear plant given as a derivative and an output map, integrated with a
/// named rule.
///
/// The derivative is `f(x, u, dt)`, taking the period as well because a plant may
/// be defined for a fixed control period; the rule treats it as the vector field
/// at `x`.
pub(all) struct Dynamics {
  f : (@linear.Vec, @linear.Vec, Double) -> @linear.Vec
  h : (@linear.Vec) -> @linear.Vec
  integrator : Integrator
}

///|
/// A nonlinear plant from its derivative and its output map, integrated with
/// RK4 unless told otherwise.
pub fn Dynamics::new(
  f~ : (@linear.Vec, @linear.Vec, Double) -> @linear.Vec,
  h~ : (@linear.Vec) -> @linear.Vec,
  integrator~ : Integrator = Rk4,
) -> Dynamics {
  { f, h, integrator, }
}

///|
/// The derivative of the plant at `x` under `u` over a nominal period `dt`.
fn Dynamics::derivative(
  self : Dynamics,
  x : @linear.Vec,
  u : @linear.Vec,
  dt : Double,
) -> @linear.Vec {
  (self.f)(x, u, dt)
}

///|
/// One period of the plant, integrated with the chosen rule.
pub impl Model for Dynamics with fn step(self, x, u, dt) {
  match self.integrator {
    Euler => x.add(self.derivative(x, u, dt).scale(dt))
    Midpoint => {
      let k1 = self.derivative(x, u, dt)
      let k2 = self.derivative(x.add(k1.scale(dt / 2.0)), u, dt)
      x.add(k2.scale(dt))
    }
    Rk4 => {
      let k1 = self.derivative(x, u, dt)
      let k2 = self.derivative(x.add(k1.scale(dt / 2.0)), u, dt)
      let k3 = self.derivative(x.add(k2.scale(dt / 2.0)), u, dt)
      let k4 = self.derivative(x.add(k3.scale(dt)), u, dt)
      let weighted = k1
        .add(k2.scale(2.0))
        .add(k3.scale(2.0))
        .add(k4)
        .scale(dt / 6.0)
      x.add(weighted)
    }
  }
}

///|
/// The measurement the output map gives.
pub impl Model for Dynamics with fn observe(self, x) {
  (self.h)(x)
}

///|
/// The integrator and the shapes, without the closures, which have no text form
/// worth printing.
impl @debug.Debug for Dynamics with fn to_repr(self) {
  let name = match self.integrator {
    Euler => "Euler"
    Midpoint => "Midpoint"
    Rk4 => "Rk4"
  }
  @debug.Repr::literal("Dynamics(" + name + ")")
}

///|
pub extend Dynamics with @debug.Debug::{to_repr}

///|
pub extend Discretize with @debug.Debug::{to_repr}

///|
pub extend StateSpace with @debug.Debug::{to_repr}

///|
pub extend Integrator with @debug.Debug::{to_repr}

///|
pub extend StateSpace with Model::{
  step,
  observe,
  step_jacobian,
  observe_jacobian,
}

///|
pub extend Dynamics with Model::{step, observe, step_jacobian, observe_jacobian}
