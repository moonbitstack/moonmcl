---
name: moonmcl
description: "Motion control for MoonBit: contracts for plants, controllers, estimators, trajectories and planners, with PID and LQR/MPC control, Kalman and particle filtering, jerk-limited trajectories, sampling-based planning over Dubins and Reeds-Shepp car spaces, and robot kinematics. Use when writing a control loop, a state estimator, a motion profile, a path planner or robot kinematics in MoonBit."
---

# moonmcl

A motion-control library built around contracts: implement `Model`, `Controller`,
`Estimator`, `Trajectory`, `Kinematics`, `Space`, `Validity` or `Planner`, and
every algorithm in the library takes your type without a change here.

```sh
moon add moonbitstack/moonmcl
```

Import the whole surface under one alias, or one package on its own
(`"moonbitstack/moonmcl/control"` and so on).

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}
```

## Run a control loop

Every controller implements `Controller`: a reference, a measurement and a
period in, a command out. The PID family also has a scalar entry point.

```moonbit
let pid = @mctl.Pid::new(config=@mctl.pid_gains(kp=2.0, ki=0.5, kd=0.1))
let mut y = 0.0
for _ in 0..<100 {
  let u = pid.command(reference=1.0, measurement=y, dt=0.02)
  y = y + 0.02 * (u - y)
}
assert_eq(y > 0.7, true)
```

Combinators take `&Controller`, so saturation, rate limiting and cascades work
on a controller you wrote yourself:

```moonbit
let raw = @mctl.Pid::new(config=@mctl.pid_gains(kp=10.0))
let limited = @mctl.Saturating::new(inner=raw, output={ low: -1.0, high: 1.0 })
let command = limited.update(@mctl.Vec::of([1.0]), @mctl.Vec::of([0.0]), 0.1)
assert_eq(command.get(0), 1.0)
```

## Filter your own plant

Give a plant `step` and `observe` and the extended, unscented and particle
filters all take it; the Jacobians come from the contract by central
differences, and a plant that knows them can override the default methods.

```moonbit
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
filter.correct(@mctl.Vec::of([1.0]))
assert_eq(filter.state().len(), 1)
```

## Plan in your own space

A space says how to draw a state, how far two are and how to walk between them;
a validity oracle says what is free. Every sampling planner is written against
those two.

```moonbit
let space : &@mctl.Space = @mctl.GridSpace::new(@mctl.Grid::new(10, 10))
let validity : &@mctl.Validity = @mctl.GridSpace::new(@mctl.Grid::new(10, 10))
let planner : &@mctl.Planner = @mctl.Rrt::new(space=space, validity=validity, step=1.0)
let path = planner.plan(
  @mctl.Vec::of([0.0, 0.0]),
  @mctl.Vec::of([9.0, 9.0]),
  { iterations: 800, seconds: None },
)
assert_eq(path.length() >= 2, true)
```

## Smooth a recorded run

A filter is causal, so its early estimates ignore the measurements that came
after them. `rts` walks a recorded run backwards and pulls every estimate towards
what the step after it implies. It takes the filtered means and covariances and
the transitions between them, returns them refined, and starts from the last
entry, which comes back untouched.

```moonbit
let states = [@mctl.Vec::of([0.0, 1.0]), @mctl.Vec::of([0.1, 1.02])]
let covariances = [@mctl.Mat::identity(2), @mctl.Mat::identity(2)]
let (smoothed, _) = @mctl.rts(
  states~,
  covariances~,
  transitions=[@mctl.Mat::of([[1.0, 0.1], [0.0, 1.0]])],
  process=@mctl.Mat::of([[0.01, 0.0], [0.0, 0.01]]),
)
assert_eq(smoothed.length(), 2)
```

## Put a continuous plant on a clock

A plant given in physical units is continuous; the filter and the predictive
controller want a discrete record. `discretize` converts it by zero-order hold —
the input held over the period, which is what a digital controller does — and by
the trapezoidal rule or forward Euler when a model was written for those. A record
that is already discrete keeps its period and refuses another.

```moonbit
let plant = @mctl.StateSpace::continuous(
  a=@mctl.Mat::of([[0.0, 1.0], [0.0, 0.0]]),
  b=@mctl.Mat::of([[0.0], [1.0]]),
  c=@mctl.Mat::of([[1.0, 0.0]]),
  d=@mctl.Mat::zeros(1, 1),
)
let held = plant.discretize(dt=0.1)
assert_eq(held.is_discrete(), true)
assert_eq(held.a.get(0, 1) > 0.09, true)
```

## Drive a car, or plan in its space

`Dubins` and `ReedsShepp` are state spaces: a pose is a state, `distance` is the
length of the shortest path the car can drive, and `interpolate` walks that path.
Use them directly for a shortest path, or hand one to a sampling planner so every
edge of the tree is drivable.

```moonbit
let car = @mctl.ReedsShepp::new(turning_radius=1.0)
let start = @mctl.Vec::of([0.0, 0.0, 0.0])
let goal = @mctl.Vec::of([-2.0, 0.8, 0.5])
let path = car.path(start~, goal~)
assert_eq(path.at(path.length).get(0).abs() < 2.000001, true)
assert_eq(path.cusps() >= 0, true)
```

```moonbit
let space : &@mctl.Space = @mctl.ReedsShepp::new(turning_radius=0.6)
let validity : &@mctl.Validity = @mctl.GridSpace::new(@mctl.Grid::new(10, 10))
let planner : &@mctl.Planner = @mctl.RrtStar::new(space=space, validity=validity, step=0.5)
let path = planner.plan(
  @mctl.Vec::of([-2.0, 0.0, 0.0]),
  @mctl.Vec::of([2.0, 0.0, 0.0]),
  { iterations: 1500, seconds: None },
)
assert_eq(path.length() >= 2, true)
```

## Rules that bite

- Units are SI and angles are radians; a plane pose is `(x, y, theta)` with
  `theta` counter-clockwise.
- Controllers and trackers keep state. Call `reset` between runs, or the
  integrator and the previous measurement carry over.
- `Vec` and `Mat` arithmetic aborts on a shape mismatch: that is a programming
  error. The routines that factorise a matrix or solve for a motion report
  `Refused` instead, because there the shape comes from data.
- `Estimator::predict` does not raise; `correct` does. A filter built from a
  model with the wrong dimensions refuses on construction.
- The modelling predictive controller needs a *discrete* `StateSpace`; a
  continuous record is refused rather than silently integrated.
- A jerk-limited profile refuses a boundary velocity that points against the
  direction of travel, and one that would have to reverse mid-move; it is written
  for one direction.
- A non-zero start or end acceleration is folded into the first or last ramp, so
  a chained move keeps the acceleration the previous one ended with instead of
  paying for a stop in acceleration.
