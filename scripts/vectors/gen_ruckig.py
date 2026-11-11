"""Generate the jerk-limited trajectory replay set from ruckig.

Reference: ruckig 0.19.4 (`pip install ruckig`), the online trajectory generation
library for jerk-limited profiles. Each case is one single-axis move with its own
limits and boundary velocities and accelerations; the expected value is the
duration ruckig reports.

The set keeps only the moves that stay in one direction. ruckig also solves moves
that overshoot into reverse — from 3 m/s to 1 m/s over half a metre, say — but a
single-axis profile here is written for motion along one direction, so those
cases are out of scope and the generator drops them.

Run from the repository root:

    python scripts/vectors/gen_ruckig.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import moonbit_source
from ruckig import InputParameter, Ruckig, Trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "trajectory", "ruckig_vectors_test.mbt")

PINNED = [
    # (distance, v_max, a_max, j_max, v0, a0, v1, a1)
    (10.0, 5.0, 10.0, 30.0, 0.0, 0.0, 0.0, 0.0),
    (1.0, 5.0, 10.0, 30.0, 0.0, 0.0, 0.0, 0.0),
    (10.0, 5.0, 10.0, 30.0, 1.0, 0.0, 2.0, 0.0),
    (2.0, 3.0, 4.0, 20.0, 1.0, 0.5, 0.0, 0.0),
    (2.0, 3.0, 4.0, 20.0, 0.0, 0.0, 1.0, -0.5),
    (3.0, 2.0, 1.5, 6.0, 0.8, 0.4, 1.2, 0.6),
    (3.0, 2.0, 1.5, 6.0, 1.2, -0.4, 0.8, -0.6),
    (0.4, 4.0, 3.0, 25.0, 0.3, 0.0, 0.6, 0.0),
    (40.0, 6.0, 2.0, 8.0, 2.0, 1.0, 2.0, -1.0),
    (0.02, 2.0, 6.0, 40.0, 0.0, 0.0, 0.0, 0.0),
]


def solve(distance, v_max, a_max, j_max, v0, a0, v1, a1):
    """The duration ruckig reports, or None when it refuses or reverses."""
    inp = InputParameter(1)
    inp.current_position = [0.0]
    inp.current_velocity = [v0]
    inp.current_acceleration = [a0]
    inp.target_position = [distance]
    inp.target_velocity = [v1]
    inp.target_acceleration = [a1]
    inp.max_velocity = [v_max]
    inp.max_acceleration = [a_max]
    inp.max_jerk = [j_max]
    otg = Ruckig(1)
    traj = Trajectory(1)
    try:
        result = otg.calculate(inp, traj)
    except Exception:
        return None
    if result < 0:
        return None
    if not math.isfinite(traj.duration):
        return None
    for k in range(201):
        _, velocity, _ = traj.at_time(traj.duration * k / 200.0)
        if velocity[0] < -1e-9:
            return None
    return traj.duration


def main():
    random.seed(20261112)
    cases = []
    for pinned in PINNED:
        duration = solve(*pinned)
        if duration is None:
            raise SystemExit("a pinned case is not solvable: %r" % (pinned,))
        cases.append(pinned + (duration,))
    tries = 0
    while len(cases) < len(PINNED) + 60 and tries < 4000:
        tries += 1
        v_max = random.uniform(0.8, 6.0)
        a_max = random.uniform(0.5, 4.0)
        j_max = random.uniform(2.0, 40.0)
        v0 = random.uniform(0.0, v_max * 0.8)
        v1 = random.uniform(0.0, v_max * 0.8)
        a0 = random.uniform(-a_max, a_max) * 0.95
        a1 = random.uniform(-a_max, a_max) * 0.95
        distance = random.uniform(0.02, 30.0)
        duration = solve(distance, v_max, a_max, j_max, v0, a0, v1, a1)
        if duration is None:
            continue
        cases.append((distance, v_max, a_max, j_max, v0, a0, v1, a1, duration))

    lines = []
    for case in cases:
        distance, v_max, a_max, j_max, v0, a0, v1, a1, duration = case
        lines.append(
            "  { distance: %s, v_max: %s, a_max: %s, j_max: %s, "
            "start_velocity: %s, start_acceleration: %s, end_velocity: %s, "
            "end_acceleration: %s, duration: %s },"
            % tuple(moonbit_source.number(v) for v in case)
        )
    body = """
///|
/// One single-axis move and the duration the reference implementation gives it.
struct RuckigCase {
  distance : Double
  v_max : Double
  a_max : Double
  j_max : Double
  start_velocity : Double
  start_acceleration : Double
  end_velocity : Double
  end_acceleration : Double
  duration : Double
}

///|
fn ruckig_cases() -> Array[RuckigCase] {
  [
%s
  ]
}
""" % "\n".join(lines)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_ruckig.py", "ruckig 0.19.4", body
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
