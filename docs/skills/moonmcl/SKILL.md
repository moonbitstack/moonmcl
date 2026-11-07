---
name: moonmcl
description: "MoonBit Motion Control Library. Use it for robot / machine / game motion control: PID and feedforward, jerk-limited S-curve trajectory planning, LQR and model-predictive control, Kalman / extended Kalman filtering, 2-D grid path planning (Dijkstra, A*), and differential-drive kinematics."
---

# moonmcl

Pure-MoonBit motion-control algorithms. Depends only on the official standard
library. Add it with:

```bash
moon add moonbitstack/moonmcl
```

Import the module you need — each is an independent package. The root package
re-exports the public API of every module, so you can also import it once under
an alias (the examples use `mctl`) and address the whole surface flat, e.g.
`@mctl.Pid`, `@mctl.Grid`, `@mctl.Vector`.

```moonbit
import {
  "moonbitstack/moonmcl" @mctl,
}

let pid = @mctl.Pid::pid(1.0, 0.2, 0.0)
assert_eq((pid.update(1.0, 0.05) - 1.01).abs() < 1.0e-9, true)
```

## Common tasks

### PID servo

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

let pid = @control.Pid::pid(1.0, 0.2, 0.0, output_limit=Some(2.0))
let out = pid.update(0.5, 0.05)
assert_eq(out > 0.0, true)
```

Call `update` once per tick with the error and the tick period. The integrator
is clamped while the output is saturated, so it does not wind up.

### S-curve move

```moonbit
import {
  "moonbitstack/moonmcl/control",
}

let profile = @control.scurve_plan(100.0, 5.0, 2.0, 1.0)
let (s, v, a, j) = profile.at(profile.total_time)
assert_eq((s - 100.0).abs() < 1.0e-6, true)
```

### LQR / MPC

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
assert_eq(k.rows == 1, true)
```

For MPC, build `@control.Mpc::mpc(a, b, q, r, horizon)` and call `step(x, xref)`
once per tick (receding horizon).

### Grid path planning

```moonbit
import {
  "moonbitstack/moonmcl/planning",
}

let grid = @planning.Grid::grid(10, 10)
let path = @planning.astar(grid, grid.index(0, 0), grid.index(9, 9))
assert_eq(path is Some(_), true)
```

### Differential-drive kinematics

```moonbit
import {
  "moonbitstack/moonmcl/kinematics",
}

let wheel = @kinematics.WheelVel::wheel_vel(0.5, 1.5)
let twist = @kinematics.forward_kinematics(wheel, 0.5)
assert_eq((twist.v - 1.0).abs() < 1.0e-12, true)
```

## Gotchas

- All angles are radians, lengths metres, time seconds; scalars are `Double`.
- `scurve_plan` requires strictly positive `v_max`, `a_max` and `j_max` (it
  aborts otherwise) and treats a negative distance as a reverse move.
- `Grid::grid`'s `blocked_cells` is an optional `Array[Int32]` of flat indices;
  pass `Some([...])` to block cells, or omit it for a fully free grid.
- Path methods return `Array[Int32]?` — `None` when the goal is unreachable or
  blocked.
