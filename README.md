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
| `scurve` | jerk-limited and time optimal from any boundary velocity and acceleration |
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
| `Dubins`, `DubinsPath`, `DubinsWord` | the forward-only car: six words, as a `Space` |
| `ReedsShepp`, `ReedsSheppPath`, `ReedsSheppSegment`, `ReedsSheppWord` | the reversing car: forty-eight words, as a `Space` |
| `DynamicWindow`, `PotentialField` | the two local planners |
| `grid_path`, `GridSearch`, `GridSpace` | the grid search of the graph package, and the grid as a `Space` |

Every planner carries a seeded generator, so a run replays exactly; `Budget`
bounds it in iterations and optionally in wall-clock seconds.

Both car models implement `Space`, so a sampling planner plans in the car's own
geometry: every edge of the tree is a drivable arc or line, and a reversing car
can back out of a corner a forward-only one cannot.

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

let car = @mctl.ReedsShepp::new(turning_radius=1.0)
let start = @mctl.Vec::of([0.0, 0.0, 0.0])
let goal = @mctl.Vec::of([-1.0, 0.4, 0.0])
let path = car.path(start~, goal~)
let end = path.at(path.length)
assert_eq((end.get(0) - -1.0).abs() < 1.0e-9, true)
assert_eq((end.get(1) - 0.4).abs() < 1.0e-9, true)
assert_eq(path.length < @mctl.Dubins::new(turning_radius=1.0).path(start~, goal~).length, true)
```


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

Each family with a mature reference replays it: the tables under
`scripts/vectors/` are produced by the scripts beside them, and `moon test` reads
them without the reference installed. Where a comparison needed a tolerance wider
than the last digits of a double, or where the two sides do not agree at all, it
is listed here.

- **The jerk-limited profile** matches the reference durations to nine digits on
  every replayed move, with the start and end accelerations folded into the first
  and last ramp. A move that would have to *reverse* mid-way — from 3 m/s to 1 m/s
  over half a metre, say — is refused: this profile is written for motion along
  one direction, and the reference solves the reversing case with a different
  structure. The refusal is part of the contract and is tested as such. A boundary
  acceleration that points against the move is carried into the first ramp, so a
  profile can begin with a short stretch of negative velocity — a few centimetres
  per second across the swept domain — before the move itself starts; that is the
  state it was handed rather than a reversal of the move.
- **Reeds-Shepp** matches the reference length, the number of cusps and the pose
  along the path. Two words of exactly equal length meet at the same place, so
  which of the two is reported can differ from the reference; both are shortest.
- **Dubins** matches the reference length and the pose along the path for all six
  words.
- **The unscented filter** is compared at `1e-7` rather than `1e-9`: the reference
  takes the matrix square root by Cholesky and this library takes it by symmetric
  eigen decomposition, so the sigma points differ in the last digits. The linear
  and extended filters agree to `1e-9`.
- **The Riccati solution** is compared at `1e-8`: the reference solves the
  equation with a Schur decomposition and this library iterates the recursion to a
  fixed point.
- **Sampling plans** are compared by cost and feasibility, not point by point: two
  planners drawing from different generators find different routes of the same
  order of magnitude, and a route is judged by whether the map allows it.
- **Arm kinematics** are compared on the forward map exactly; the inverse is
  judged by where the solved angles put the tip, because a chain with more than
  two links has more than one set of angles for the same tip.
- **Dense linear algebra** matches the reference solve, inverse, determinant,
  Cholesky factor, symmetric eigenvalues and matrix exponential to `1e-9`. A QR
  factorisation is unique only up to the sign of each column, so both sides
  orient the columns the same way before comparing, at `1e-10`.
- **The default numerical Jacobians** are compared at `1e-8`. The reference
  differentiates by complex step, which is accurate to machine precision; the
  contracts use a central difference with a step of `1e-6`, so the difference is
  the truncation and cancellation error of that step.
- **Discretisation** matches the reference for all three rules to `1e-12` — the
  zero-order hold, the trapezoidal rule and forward Euler — on plants from a
  double integrator to a three-state chain and random matrices whose spectrum sits
  in the left half plane. The hold is the one that matters downstream, and it is
  the matrix exponential of the augmented plant, so the comparison covers both the
  augmentation and `expm`.
- **The backward smoothing pass** is replayed against the reference on its own
  forward record: the filtered means, covariances and transitions are the ones the
  reference held, and only the smoothed output is compared, at `1e-9`. That
  separates a smoother that disagrees from a filter that does.

## Licence

Apache-2.0.
