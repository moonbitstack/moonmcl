# MoonMCL — MoonBit Motion Control Library

<p>
  <a href="https://moonbitstack.github.io/moonmcl"><b>Home</b></a> &nbsp;·&nbsp;
  <a href="https://moonbitstack.github.io/moonmcl/guide.html"><b>Guide</b></a> &nbsp;·&nbsp;
  <a href="https://moonbitstack.github.io/moonmcl/api.html"><b>API reference</b></a> &nbsp;·&nbsp;
  <a href="https://mooncakes.io/docs/moonbitstack/moonmcl"><b>mooncakes</b></a>
</p>

Motion control for robots, machines and simulations, written in MoonBit and laid
out contract first: a plant, a controller, an estimator, a trajectory, a
mechanism and a planner are each a trait, and every algorithm in the library is
written against those traits rather than against a concrete type. Implement one
of them for your own type and the filters, the combinators, the synchroniser and
the sampling planners take it — without a change to this repository.

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

// A jerk-limited move over twelve metres.
let profile = @mctl.scurve(distance=12.0, v_max=2.0, a_max=1.0, j_max=4.0)
let end = profile.sample(profile.duration())
assert_eq((end.position - 12.0).abs() < 1.0e-6, true)
assert_eq(end.velocity.abs() < 1.0e-6, true)
```

Install it and run the examples:

```sh
moon add moonbitstack/moonmcl
moon run src/examples/01-pid
```

## The contracts

| Trait | What a caller supplies | Who uses it |
|:--|:--|:--|
| `Model` | `step(x, u, dt)`, `observe(x)`, and optionally the two Jacobians | extended, unscented and particle filters; iterative LQR |
| `Controller` | `update(reference, measurement, dt)`, `reset` | cascade, saturation and rate limiting |
| `Estimator` | `predict(u, dt)`, `correct(z)`, `state`, `covariance` | anything that wants a filtered state |
| `Trajectory` | `duration`, `sample(t)` | multi-axis synchronisation |
| `Kinematics` | `forward(q)`, `inverse(x, seed?)`, optionally `jacobian(q)` | computed torque, Jacobian-based inverse kinematics |
| `Space` | `sample(rng)`, `distance(a, b)`, `interpolate(a, b, t)` | every sampling planner, Dubins, the grid |
| `Validity` | `valid(x)`, optionally `motion(space, a, b)` | every sampling planner |
| `Planner` | `plan(start, goal, budget)` | — |

The Jacobians are default methods: a plant that gives only `step` and `observe`
gets a central difference with the published step `jacobian_step`, and one that
knows its analytic Jacobian overrides the same method. `Validity::motion` is a
default too, walking the space's own interpolation in `motion_samples` steps.

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

let plant = @mctl.Dynamics::new(
  f=fn (x, u, dt) {
    ignore(u)
    @mctl.Vec::of([x.get(0) + dt])
  },
  h=fn (x) { @mctl.Vec::of([x.get(0)]) },
)
let filter = @mctl.ExtendedKalman::new(
  model=plant,
  process=@mctl.Mat::of([[0.01]]),
  measurement=@mctl.Mat::of([[0.1]]),
  state=@mctl.Vec::of([0.0]),
  covariance=@mctl.Mat::of([[1.0]]),
)
filter.predict(@mctl.Vec::of([0.0]), 1.0)
filter.correct(@mctl.Vec::of([2.0]))
assert_eq(filter.state().len(), 1)
```

## Packages

Every package can be imported on its own (`"moonbitstack/moonmcl/control"`,
`"moonbitstack/moonmcl/planning"`, …) or through the root facade under any
alias. The examples use `mctl`.

### `linear`

Dense vectors and matrices in double precision, and the factorisations the rest
of the library leans on. A `Vec` is a column of `Double`; a `Mat` is row-major
with `rows`, `cols` and `data`.

| Item | What it does |
|:--|:--|
| `Vec::zeros`, `Vec::of`, `Vec::linspace`, `Vec::filled` | construction |
| `Vec::add`, `sub`, `hadamard`, `scale`, `axpy`, `dot`, `norm`, `normalized` | arithmetic |
| `Mat::zeros`, `Mat::identity`, `Mat::of`, `Mat::diagonal`, `Mat::from_fixed` | construction |
| `Mat::mul`, `Mat::apply`, `Mat::apply_transposed`, `Mat::transpose`, `Mat::augment`, `Mat::stack`, `Mat::block` | products and assembly |
| `solve`, `solve_vec`, `inv`, `det`, `lu`, `qr`, `cholesky`, `cholesky_solve`, `eigh`, `norm`, `expm` | factorisations, named as in `numpy.linalg` |

`lu` keeps the factorisation so several right-hand sides share it; `eigh` is the
symmetric eigendecomposition by Jacobi rotations with ascending eigenvalues;
`expm` is scaling and squaring, which is what turns a continuous state matrix
into a discrete one.

Errors are one type, `Refused`, with a variant per kind of failure: `Dimension`,
`Empty`, `Singular(at~)`, `NotPositiveDefinite(at~)`, `Gain(got~)`,
`Range(got~, low~, high~)`, `Exceeded(limit~, got~)` and `Unreachable`.

### `contract`

The eight traits, plus `Sample` (position, velocity, acceleration, jerk),
`Budget` (iterations, optional wall clock), `StateSpace`, `Dynamics`,
`Discretize` and `Integrator`.

`StateSpace` is a linear plant: `continuous(a~, b~, c~, d~)` or
`discrete(a~, b~, c~, d~, dt~)`, and `discretize(dt~, discretize~ = Zoh)` turns
the first into the second by zero-order hold — the input held over the period,
which is what a digital controller actually does. `Dynamics` is a nonlinear
plant from two functions and an integrator (`Euler`, `Midpoint`, `Rk4`, the
preset). Both implement `Model`.

### `control`

| Item | What it does |
|:--|:--|
| `Pid`, `PidConfig`, `pid_gains`, `pid_config` | conditional and back-calculation anti-windup, setpoint weighting, derivative on the measurement, a first-order filter on it |
| `Feedforward`, `FeedforwardRunner` | velocity and acceleration feedforward, on a sample or on a reference sequence |
| `Cascade`, `Saturating`, `RateLimited` | combinators over `&Controller` |
| `dare`, `Riccati`, `lqr`, `Lqr` | the discrete Riccati equation and the infinite-horizon gain |
| `integral_lqr`, `IntegralLqr` | the gain with an integrator on the output error |
| `ilqr`, `IlqrConfig` | iterative LQR over any `Model` |
| `Mpc`, `MpcConfig`, `box_qp`, `QpConfig` | receding-horizon control with box limits on the input and on its rate, and the quadratic program underneath |
| `Pursuit`, `Stanley` | pure pursuit and Stanley on a polyline |
| `ComputedTorque`, `Impedance` | model-based effort |
| `cbf_filter`, `CbfConfig` | the closest safe input to a nominal one |

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

// Saturation, rate limiting and a cascade all take any controller.
let raw = @mctl.Pid::new(config=@mctl.pid_gains(kp=10.0))
let limited = @mctl.Saturating::new(inner=raw, output={ low: -1.0, high: 1.0 })
let command = limited.update(@mctl.Vec::of([1.0]), @mctl.Vec::of([0.0]), 0.1)
assert_eq(command.get(0), 1.0)
```

The quadratic program is solved by ADMM: the horizon of a predictive controller
is re-solved every period with the same Hessian, so a method whose iteration is
one triangular solve and one clamp keeps the period short and has no active set
to warm start.

### `estimation`

| Item | What it does |
|:--|:--|
| `Kalman` | the linear filter for a discrete `StateSpace`, Joseph form |
| `ExtendedKalman`, `UnscentedKalman`, `UnscentedConfig` | the same recursion over any `Model`, linearised or with sigma points |
| `ParticleFilter` | a cloud of hypotheses, weighted by the innovation and resampled when the effective size halves |
| `Complementary`, `AlphaBetaGamma` | the two single-axis trackers |
| `rts` | the Rauch-Tung-Striebel backward pass |

### `trajectory`

| Item | What it does |
|:--|:--|
| `trapezoid` | accelerate, cruise, decelerate, in closed form |
| `scurve` | jerk-limited, seven phases, boundary velocities and accelerations |
| `cubic`, `quintic` | polynomials from their boundary states |
| `cubic_spline` | natural cubic through time-stamped knots |
| `b_spline` | clamped B-spline, evaluated by de Boor |
| `Synced` | several axes on one clock |

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

let fast = @mctl.trapezoid(distance=1.0, v_max=1.0, a_max=1.0)
let far = @mctl.trapezoid(distance=4.0, v_max=1.0, a_max=1.0)
let set = @mctl.Synced::new(axes=[fast, far])
let ends = set.sample_all(set.duration())
assert_eq((ends[0].position - 1.0).abs() < 1.0e-9, true)
```

### `planning`

| Item | What it does |
|:--|:--|
| `Rrt`, `RrtStar`, `RrtConnect`, `InformedRrtStar` | sampling planners over any `Space` and `Validity` |
| `Prm` | a roadmap built once and queried many times |
| `Dubins`, `DubinsPath`, `DubinsWord` | the forward-only car, as a `Space` |
| `DynamicWindow`, `PotentialField` | the two local planners |
| `grid_path`, `GridSearch`, `GridSpace` | the grid search of the graph package, and the grid as a `Space` |

Every planner carries a seeded generator, so a run replays exactly; `Budget`
bounds it in iterations and optionally in wall-clock seconds.

### `kinematics`

| Item | What it does |
|:--|:--|
| `Differential` | two wheels to a body twist, and the exact arc for the pose |
| `Bicycle`, `Ackermann` | steering bases, with the inner and outer angles |
| `Mecanum`, `Omnidirectional` | wheels that push sideways; the inverse is the least-squares one |
| `PlanarArm` | an `n`-link chain with an analytic Jacobian and a damped-least-squares inverse |
| `Scara`, `Delta`, `Gantry` | the three structures |

## Units and conventions

SI throughout: metres, seconds, radians. A plane pose is `(x, y, theta)` with
`theta` counter-clockwise. A wheeled base's joint space is the wheel command and
its task space the body twist, because that is what a drive commands; integrating
the twist into a pose is a separate, explicit step (`Differential::advance`).

## Consistency

Where an algorithm has a published reference, the tests pin it to one.

- The **jerk-limited profile** is checked against a published time-optimal
  profile generator: for a move that cruises, one that cannot, and a reverse
  move, the durations agree to twelve digits, and it also matches for non-zero
  boundary velocities. A non-zero boundary *acceleration* is accepted and
  honoured exactly — the start and the end states are what the caller asked for
  and the jerk limit holds throughout — but the profile it produces is not
  proven time-optimal, because the acceleration is taken off in a segment of its
  own instead of being folded into the first ramp.
- The **iterative LQR** is checked against the optimum of the same problem
  stacked into one quadratic program over the whole horizon; the two agree to
  six digits on the control sequence and on the terminal state.
- The **Riccati recursion** is checked against its own fixed-point equation to
  the tolerance the iteration stops at, and the LQR closed loop is checked to
  drive a double integrator to its reference.
- The **Kalman filter** is checked to track a constant velocity from position
  measurements and to keep its covariance symmetric and shrinking; the extended,
  unscented and particle filters are all checked to follow the same pendulum,
  whose model is written in the tests alone.
- **Dubins** paths are checked to start and end at the poses they were given and
  never to be shorter than the straight line; all six words are enumerated and
  the shortest kept.
- **Reeds-Shepp is not implemented.** A reversing car is a different set of
  forty-eight words and it is not in this version; `Dubins` covers the
  forward-only case.
- The filters, the planners and the mechanisms are checked against their own
  algebra and against simulation, not against an external vector set; the tests
  that would do that are the next thing to add.

## Licence

Apache-2.0.
