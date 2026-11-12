"""Generate a second opinion on both car models, from PythonRobotics.

The primary tables for these two families come from `dubins` 1.0.1 and rsplan
1.0.10. Both are also implemented in PythonRobotics, in files written separately
from either: `PathPlanning/DubinsPath/dubins_path_planner.py` for the forward-only
car and `PathPlanning/ReedsSheppPath/reeds_shepp_path_planning.py` for the
reversing one. The cases here are a fresh draw, not the ones the primary tables
replay, so agreement means two independent implementations agree with this
library rather than one agreeing with itself.

Only the length is recorded. The primary tables already pin the poses along the
path and the cusp count, and the length is the quantity both references solve for
exactly, so it is the one that can be compared at 1e-9 without arguing about how
either side numbers its segments.

Run from the repository root:

    python scripts/vectors/gen_car_pythonrobotics.py --robotics ~/PythonRobotics
"""

import argparse
import math
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import moonbit_source

PLANNING = os.path.join(HERE, "..", "..", "src", "planning")
DUBINS_OUT = os.path.join(PLANNING, "dubins_pythonrobotics_vectors_test.mbt")
SHEPP_OUT = os.path.join(PLANNING, "reeds_shepp_pythonrobotics_vectors_test.mbt")


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def dubins_length(module, start, goal, radius):
    """The length PythonRobotics reports, and the pose its own walk ends on."""
    xs, ys, yaw, _, lengths = module.plan_dubins_path(
        start[0], start[1], start[2], goal[0], goal[1], goal[2],
        1.0 / radius, step_size=0.05,
    )
    return sum(lengths), (xs[-1], ys[-1], yaw[-1])


def shepp_length(module, start, goal, radius):
    xs, ys, yaw, _, lengths = module.reeds_shepp_path_planning(
        start[0], start[1], start[2], goal[0], goal[1], goal[2],
        1.0 / radius, step_size=0.05,
    )
    if not lengths:
        return None, None
    # A reversing segment is reported with a negative length, so the distance
    # driven is the sum of the magnitudes.
    return sum(abs(v) for v in lengths), (xs[-1], ys[-1], yaw[-1])


def landed(pose, goal, tolerance=1e-2):
    """Whether the reference's own walk ends on the goal it was given."""
    if pose is None:
        return False
    dx = pose[0] - goal[0]
    dy = pose[1] - goal[1]
    dyaw = wrap(pose[2] - goal[2])
    return math.hypot(dx, dy) < tolerance and abs(dyaw) < tolerance


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robotics", default=os.path.expanduser("~/PythonRobotics"))
    args = parser.parse_args()
    sys.path.insert(0, os.path.abspath(args.robotics))

    from PathPlanning.DubinsPath import dubins_path_planner as dubins
    from PathPlanning.ReedsSheppPath import reeds_shepp_path_planning as shepp

    rng = random.Random(20261113)
    dubins_cases = []
    while len(dubins_cases) < 24:
        radius = rng.uniform(0.4, 2.0)
        goal = [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)]
        start = [0.0, 0.0, 0.0]
        length, pose = dubins_length(dubins, start, goal, radius)
        if not landed(pose, goal):
            continue
        dubins_cases.append((start, goal, radius, length))

    shepp_cases = []
    attempts = 0
    while len(shepp_cases) < 24 and attempts < 400:
        attempts += 1
        radius = rng.uniform(0.5, 2.0)
        goal = [rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0), rng.uniform(-3.0, 3.0)]
        start = [0.0, 0.0, 0.0]
        length, pose = shepp_length(shepp, start, goal, radius)
        if length is None or not landed(pose, goal):
            continue
        shepp_cases.append((start, goal, radius, length))

    dubins_blocks = []
    for start, goal, radius, length in dubins_cases:
        dubins_blocks.append(
            "  {\n    start: %s,\n    goal: %s,\n    radius: %s,\n    length: %s,\n  },"
            % (
                moonbit_source.array(start),
                moonbit_source.array(goal),
                moonbit_source.number(radius),
                moonbit_source.number(length),
            )
        )
    dubins_body = """
///|
/// One pair of poses, a turning radius, and the length PythonRobotics drives
/// between them.
struct DubinsCrossCase {
  start : Array[Double]
  goal : Array[Double]
  radius : Double
  length : Double
}

///|
fn dubins_cross_cases() -> Array[DubinsCrossCase] {
  [
%s
  ]
}
""" % "\n".join(dubins_blocks)
    moonbit_source.write(
        DUBINS_OUT,
        moonbit_source.bundle(
            "scripts/vectors/gen_car_pythonrobotics.py",
            "PythonRobotics `plan_dubins_path`",
            dubins_body,
        ),
    )

    shepp_blocks = []
    for start, goal, radius, length in shepp_cases:
        shepp_blocks.append(
            "  {\n    start: %s,\n    goal: %s,\n    radius: %s,\n    length: %s,\n  },"
            % (
                moonbit_source.array(start),
                moonbit_source.array(goal),
                moonbit_source.number(radius),
                moonbit_source.number(length),
            )
        )
    shepp_body = """
///|
/// One pair of poses, a turning radius, and the length PythonRobotics drives
/// between them with the car allowed to reverse.
struct ReedsSheppCrossCase {
  start : Array[Double]
  goal : Array[Double]
  radius : Double
  length : Double
}

///|
fn reeds_shepp_cross_cases() -> Array[ReedsSheppCrossCase] {
  [
%s
  ]
}
""" % "\n".join(shepp_blocks)
    moonbit_source.write(
        SHEPP_OUT,
        moonbit_source.bundle(
            "scripts/vectors/gen_car_pythonrobotics.py",
            "PythonRobotics `reeds_shepp_path_planning`",
            shepp_body,
        ),
    )
    print("%d dubins, %d reeds shepp" % (len(dubins_cases), len(shepp_cases)))


if __name__ == "__main__":
    main()
