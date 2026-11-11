"""Generate the Reeds-Shepp replay set from rsplan.

Reference: rsplan 1.0.10 (`pip install rsplan`, Built Robotics' implementation of
the forty-eight words). Each case is a start pose, a goal pose and a turning
radius; the expected values are the shortest length, the number of cusps, and the
pose at even distances along the path.

The reference is asked for the shortest path without the "fewer segments within
two metres" preference its planner carries by default (`length_tolerance=0`).

Run from the repository root:

    python scripts/vectors/gen_reeds_shepp.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rsplan

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(
    HERE, "..", "..", "src", "planning", "reeds_shepp_vectors_test.mbt"
)

SAMPLES = 7


def wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def case(start, goal, radius):
    path = rsplan.path(
        start_pose=start,
        end_pose=goal,
        turn_radius=radius,
        runway_length=0.0,
        step_size=0.05,
        length_tolerance=0.0,
    )
    length = path.total_length
    xs, ys, headings = path.coordinates_tuple()
    # The reference walks each piece in evenly spaced points, so the arc length of
    # every waypoint is the length of the pieces before it plus the steps into
    # this one; a sample carries that length, which is what this library's own
    # sampling takes.
    step_size = 0.05
    walked = []
    walked_total = 0.0
    for index, segment in enumerate(path.segments):
        piece = abs(segment.length)
        magnitude = piece if segment.is_straight else piece / radius
        step = step_size if segment.is_straight else step_size / radius
        count = int(magnitude / step + 2)
        stride = piece / (count - 1)
        for k in range(count):
            if index > 0 and k == 0:
                continue
            walked.append(walked_total + k * stride)
        walked_total += piece
    assert len(walked) == len(xs), (len(walked), len(xs))
    samples = []
    for k in range(SAMPLES):
        wanted = length * k / (SAMPLES - 1.0)
        index = min(range(len(xs)), key=lambda i: abs(walked[i] - wanted))
        samples.append([walked[index], xs[index], ys[index], wrap(headings[index])])
    return {
        "start": list(start),
        "goal": list(goal),
        "radius": radius,
        "length": length,
        "cusps": int(path.number_of_cusp_points),
        "samples": samples,
    }


def main():
    rng = random.Random(20261112)
    cases = [
        case((0.0, 0.0, 0.0), (2.0, 1.0, 1.0), 1.0),
        case((0.0, 0.0, 0.0), (-2.0, 0.0, 0.0), 1.0),
        case((0.0, 0.0, 0.0), (-1.0, 0.4, 0.0), 1.0),
        case((1.0, -2.0, 0.7), (-1.5, 2.0, -0.4), 0.6),
        case((-2.0, -1.0, 3.0), (2.0, 1.0, -3.0), 1.5),
        case((0.5, 0.5, 0.2), (0.6, 0.4, 3.0), 2.0),
    ]
    while len(cases) < 30:
        radius = rng.uniform(0.3, 3.0)
        start = (rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-3.1, 3.1))
        goal = (rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-3.1, 3.1))
        cases.append(case(start, goal, radius))

    blocks = []
    for item in cases:
        sample_lines = ",\n".join(
            "      " + moonbit_source.array(s) for s in item["samples"]
        )
        blocks.append(
            """  {
    start: %s,
    goal: %s,
    radius: %s,
    length: %s,
    cusps: %d,
    samples: [
%s
    ],
  }"""
            % (
                moonbit_source.array(item["start"]),
                moonbit_source.array(item["goal"]),
                moonbit_source.number(item["radius"]),
                moonbit_source.number(item["length"]),
                item["cusps"],
                sample_lines,
            )
        )
    body = """
///|
/// One Reeds-Shepp problem: two poses, a radius, and the shortest path between
/// them the reference draws.
struct ReedsSheppCase {
  start : Array[Double]
  goal : Array[Double]
  radius : Double
  length : Double
  cusps : Int32
  samples : Array[Array[Double]]
}

///|
fn reeds_shepp_cases() -> Array[ReedsSheppCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_reeds_shepp.py", "rsplan 1.0.10", body
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
