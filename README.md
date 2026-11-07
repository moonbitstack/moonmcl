# MoonMCL — MoonBit Motion Control Library

A pure-MoonBit collection of motion-control algorithms for robots, machines,
game characters and physics integrations: classical control (PID with
anti-windup, feedforward, jerk-limited S-curve profiles), modern control (LQR,
integral LQR, linear model-predictive control), state estimation (Kalman and
extended Kalman filters), grid path planning (Dijkstra and A*), and
differential-drive kinematics. It depends only on the official standard
library, and every module is an independent package that can be imported on its
own.

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

// A proportional-integral controller driving a first-order plant to a setpoint.
let pid = @control.Pid::pid(1.0, 0.2, 0.0, output_limit=Some(2.0))
let error = 1.0 - 0.0
assert_eq((pid.update(error, 0.05) - 1.01).abs() < 1.0e-9, true)
```

Run `moon run src/examples/01-pid` (and the rest of `src/examples/`) to see the
whole surface in motion.

## Modules

| Package | What it gives you |
|:--|:--|
| `@moonmcl.linear` | A lightweight `Vector` / `Matrix` (Double) with products, transpose, LU solve, inverse, Kronecker and QR — the basis the rest of the library builds on |
| `@moonmcl.control` | PID (anti-windup, feedforward), feedforward compensator, jerk-limited S-curve profiling, LQR, integral LQR, linear MPC, and the pure-pursuit / Stanley path followers |
| `@moonmcl.estimation` | Linear Kalman filter and extended Kalman filter |
| `@moonmcl.planning` | `Grid` plus Dijkstra and A* shortest-path search (4-connected, unit cost) |
| `@moonmcl.kinematics` | Differential-drive forward / inverse kinematics and a pose update |

### The root facade and the `mctl` alias

The root package re-exports the public API of every module, so you can import
the whole library once, under any alias, and address it flat. The examples use
`mctl`:

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

// The re-exported surface: no per-module import needed.
let pid = @mctl.Pid::pid(1.0, 0.2, 0.0, output_limit=Some(2.0))
let grid = @mctl.Grid::grid(5, 5)
let wheel = @mctl.WheelVel::wheel_vel(0.5, 1.5)
let path = @mctl.astar(grid, grid.index(0, 0), grid.index(4, 4)).unwrap()
let _ = path.length()
assert_eq((pid.update(1.0, 0.05) - 1.01).abs() < 1.0e-9, true)
```

Each module remains an independent package, so importing just one
(`"moonbitstack/moonmcl/control"`) is still available when you only need that
surface.

### Conventions

- Scalars are `Double`. All angles are radians, lengths are metres, time is
  seconds.
- Controllers are stateful: call their `update` / `step` / `predict` / `update`
  method once per control tick.
- `moon run src/examples/<n>-<topic>` runs each example.

## PID

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

let pid = @control.Pid::pid(2.0, 0.5, 0.1, output_limit=Some(10.0))
let out = pid.update(3.0, 0.05)
assert_eq(out > 0.0, true)
```

The integrator is held while the output is saturated and the error would drive
it further into saturation, so a large, long error does not wind the integrator
up and overshoot. `update` differentiates the error; `update_signals(setpoint,
measurement, dt)` differentiates the measurement instead (derivative-on-
measurement), so a step in the setpoint does not cause a derivative kick.

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

let pid = @control.Pid::pid(0.0, 0.0, 1.0)
// A setpoint step with a flat measurement produces no derivative term.
ignore(pid.update_signals(0.0, 0.0, 0.1))
assert_eq(pid.update_signals(2.0, 0.0, 0.1), 0.0)
```

## S-curve (jerk-limited) profiling

`@control.scurve_plan` builds the minimal-time rest-to-rest profile for a
distance and a peak velocity / acceleration / jerk. `at` returns the exact
position, velocity, acceleration and jerk at any instant.

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

let profile = @control.scurve_plan(100.0, 5.0, 2.0, 1.0)
let (s_end, v_end, a_end, _) = profile.at(profile.total_time)
assert_eq((s_end - 100.0).abs() < 1.0e-6, true)
assert_eq((v_end).abs() < 1.0e-6, true)
```

## LQR and MPC

`@control.lqr_gain` solves the discrete algebraic Riccati equation for a
`x_{k+1} = A x_k + B u_k` system; `@control.Mpc::mpc` runs a receding-horizon
linear MPC over a horizon, with optional input saturation.

```moonbit
import {
  "moonbitstack/moonmcl/control",
  "moonbitstack/moonmcl/linear",
}

let a = @linear.Matrix::of([[1.0]])
let b = @linear.Matrix::of([[1.0]])
let q = @linear.Matrix::of([[1.0]])
let r = @linear.Matrix::of([[1.0]])
let k = @control.lqr_gain(a, b, q, r)
// Single integrator, Q = R = 1: K is the reciprocal of the golden ratio.
assert_eq((k.at(0, 0).unwrap() - 0.6180339887).abs() < 1.0e-9, true)
```

## Kalman filter

```moonbit
import {
  "moonbitstack/moonmcl/estimation",
  "moonbitstack/moonmcl/linear",
}

let kf = @estimation.Kf::kf(
  @linear.Vector::of([0.0]),
  @linear.Matrix::of([[1.0]]),
  @linear.Matrix::of([[1.0]]),
  @linear.Matrix::of([[0.01]]),
)
kf.predict()
kf.update(@linear.Vector::of([2.0]), @linear.Matrix::of([[1.0]]), @linear.Matrix::of([[0.1]]))
assert_eq((kf.state().x() - 2.0).abs() < 0.5, true)
```

## Path planning

```moonbit
import {
  "moonbitstack/moonmcl/planning",
}

let grid = @planning.Grid::grid(10, 10)
let path = @planning.astar(grid, grid.index(0, 0), grid.index(9, 9)).unwrap()
assert_eq(path.length() - 1, 18)
```

## Differential-drive kinematics

```moonbit
import {
  "moonbitstack/moonmcl/kinematics",
}

let wheel = @kinematics.WheelVel::wheel_vel(0.5, 1.5)
let twist = @kinematics.forward_kinematics(wheel, 0.5)
assert_eq((twist.v - 1.0).abs() < 1.0e-12, true)
assert_eq((twist.w - 2.0).abs() < 1.0e-12, true)
```

## Dependencies

Only the official standard library (`moonbitlang/core`); no third-party
packages.

## Licence

Apache-2.0.
