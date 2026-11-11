"""Generate the Dubins replay set from the `dubins` package.

Reference: dubins 1.0.1 (Andrew Walker's `dubins` on PyPI, a C implementation of
Shkel and Lumelsky's word table). Each case is a start pose, a goal pose and a
turning radius; the expected values are the shortest length and the poses along
it, sampled at even distances.

Comparing poses rather than the word is deliberate: the word is not exposed by the
reference in a form that reads back, and two words of equal length meet at the
same place, so the poses along the path pin it down anyway.

The package only builds against Python 3.8, so run this with that interpreter:

    python3.8 scripts/vectors/gen_dubins.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import dubins

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "planning", "dubins_vectors_test.mbt")

SAMPLES = 9


def wrap(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def case(start, goal, radius):
    path = dubins.shortest_path(start, goal, radius)
    length = path.path_length()
    samples = []
    for k in range(SAMPLES):
        x, y, heading = path.sample(length * k / (SAMPLES - 1.0))
        samples.append([x, y, wrap(heading)])
    return {
        "start": list(start),
        "goal": list(goal),
        "radius": radius,
        "length": length,
        "samples": samples,
    }


def main():
    rng = random.Random(20261112)
    cases = [
        case((0.0, 0.0, 0.0), (2.0, 1.0, 1.0), 1.0),
        case((0.0, 0.0, 0.0), (5.0, 0.0, 0.0), 2.0),
        case((0.0, 0.0, 0.0), (3.0, 0.5, -2.5), 1.2),
        case((1.0, -2.0, 0.7), (-1.5, 2.0, -0.4), 0.6),
        case((-2.0, -1.0, 3.0), (2.0, 1.0, -3.0), 1.5),
        case((0.0, 0.0, 0.0), (0.0, 0.0, 2.0), 1.0),
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
    samples: [
%s
    ],
  }"""
            % (
                moonbit_source.array(item["start"]),
                moonbit_source.array(item["goal"]),
                moonbit_source.number(item["radius"]),
                moonbit_source.number(item["length"]),
                sample_lines,
            )
        )
    body = """
///|
/// One Dubins problem: two poses, a radius, and the shortest path between them as
/// the reference draws it.
struct DubinsCase {
  start : Array[Double]
  goal : Array[Double]
  radius : Double
  length : Double
  samples : Array[Array[Double]]
}

///|
fn dubins_cases() -> Array[DubinsCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_dubins.py", "dubins 1.0.1", body
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
