"""Generate the sampling-planner and kinematic replay sets from PythonRobotics.

Reference: https://github.com/AtsushiSakai/PythonRobotics (the checkout this
script was written against is pinned in the header of the generated file).

Two tables:

- `pythonrobotics_vectors_test.mbt` in `planning`: a small map of circular
  obstacles, the route the reference RRT* finds through it, and the cost of that
  route. The two implementations draw random samples from different generators,
  so the routes are not expected to agree point for point; what is compared is the
  order of magnitude of the cost and the feasibility of the route, and the
  comparison is made by a test that plans again with a fixed seed.

- `pythonrobotics_vectors_test.mbt` in `kinematics`: forward kinematics of planar
  n-link arms, plus joint angles from the reference's own solve, so the inverse
  in this library can be checked against the reference's forward map.

Run from the repository root:

    python scripts/vectors/gen_pythonrobotics.py --robotics ~/PythonRobotics
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
OUT_PLANNING = os.path.join(ROOT, "src", "planning", "pythonrobotics_vectors_test.mbt")
OUT_ARMS = os.path.join(ROOT, "src", "kinematics", "pythonrobotics_vectors_test.mbt")

MAPS = [
    {
        "name": "two gates",
        "start": [1.0, 1.0],
        "goal": [9.0, 9.0],
        "obstacles": [
            (3.0, 3.0, 1.2),
            (6.5, 6.5, 1.4),
            (5.0, 2.0, 0.9),
            (2.0, 7.0, 0.9),
        ],
    },
    {
        "name": "one wall",
        "start": [1.0, 5.0],
        "goal": [9.0, 5.0],
        "obstacles": [
            (5.0, 1.3, 1.0),
            (5.0, 2.8, 1.0),
            (5.0, 7.0, 1.0),
            (5.0, 8.5, 1.0),
        ],
    },
    {
        "name": "a bend",
        "start": [1.0, 1.0],
        "goal": [9.0, 8.0],
        "obstacles": [
            (2.5, 4.0, 1.0),
            (4.5, 5.5, 1.0),
            (6.5, 4.5, 1.0),
            (7.5, 7.0, 1.0),
            (4.0, 8.5, 1.0),
        ],
    },
]


def route_cost(path):
    total = 0.0
    for i in range(1, len(path)):
        total += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    return total


def plan_reference(robotics, item, seed=1):
    sys.path.insert(0, os.path.join(robotics, "PathPlanning", "RRTStar"))
    sys.path.insert(0, os.path.join(robotics, "PathPlanning"))
    from RRTStar.rrt_star import RRTStar  # noqa: E402

    random.seed(seed)
    planner = RRTStar(
        start=item["start"],
        goal=item["goal"],
        obstacle_list=item["obstacles"],
        rand_area=[0.0, 10.0],
        expand_dis=1.0,
        path_resolution=0.2,
        goal_sample_rate=10,
        max_iter=600,
        connect_circle_dist=3.0,
        search_until_max_iter=True,
        robot_radius=0.4,
    )
    path = planner.planning(animation=False)
    if path is None:
        raise SystemExit("the reference found no route for %s" % item["name"])
    return path, route_cost(path)


def arm_points(robotics, lengths, angles):
    sys.path.insert(0, os.path.join(robotics, "ArmNavigation", "n_joint_arm_to_point_control"))
    from NLinkArm import NLinkArm  # noqa: E402

    arm = NLinkArm(
        link_lengths=list(lengths),
        joint_angles=list(angles),
        goal=[0.0, 0.0],
        show_animation=False,
    )
    return arm.points[-1]


def main():
    robotics = None
    for index, argument in enumerate(sys.argv):
        if argument == "--robotics":
            robotics = sys.argv[index + 1]
    if robotics is None:
        robotics = os.path.expanduser("~/PythonRobotics")
    import matplotlib

    matplotlib.use("Agg")

    blocks = []
    for item in MAPS:
        path, cost = plan_reference(robotics, item)
        obstacle_lines = ",\n".join(
            "      " + moonbit_source.array(obstacle) for obstacle in item["obstacles"]
        )
        blocks.append(
            """  {
    name: "%s",
    start: %s,
    goal: %s,
    obstacles: [
%s
    ],
    cost: %s,
  }"""
            % (
                item["name"],
                moonbit_source.array(item["start"]),
                moonbit_source.array(item["goal"]),
                obstacle_lines,
                moonbit_source.number(cost),
            )
        )
    planning_body = """
///|
/// One planning problem: a start, a goal, circular obstacles, and the cost of the
/// route the reference finds through them.
struct SamplingCase {
  name : String
  start : Array[Double]
  goal : Array[Double]
  obstacles : Array[Array[Double]]
  cost : Double
}

///|
fn pythonrobotics_planning_cases() -> Array[SamplingCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    moonbit_source.write(
        OUT_PLANNING,
        moonbit_source.bundle(
            "scripts/vectors/gen_pythonrobotics.py", "PythonRobotics", planning_body
        ),
    )

    rng = random.Random(20261112)
    arms = []
    for lengths in ([1.0, 1.0], [1.0, 0.8, 0.6], [0.9, 0.7, 0.5, 0.4]):
        for _ in range(4):
            angles = [rng.uniform(-2.5, 2.5) for _ in lengths]
            tip = arm_points(robotics, lengths, angles)
            arms.append((lengths, angles, tip))
    arm_blocks = []
    for lengths, angles, tip in arms:
        arm_blocks.append(
            """  { links: %s, angles: %s, tip: %s }"""
            % (
                moonbit_source.array(lengths),
                moonbit_source.array(angles),
                moonbit_source.array(tip),
            )
        )
    arms_body = """
///|
/// One configuration of a planar arm and where the reference puts its tip.
struct ArmCase {
  links : Array[Double]
  angles : Array[Double]
  tip : Array[Double]
}

///|
fn pythonrobotics_arm_cases() -> Array[ArmCase] {
  [
%s
  ]
}
""" % ",\n".join(arm_blocks)
    moonbit_source.write(
        OUT_ARMS,
        moonbit_source.bundle(
            "scripts/vectors/gen_pythonrobotics.py", "PythonRobotics", arms_body
        ),
    )


if __name__ == "__main__":
    main()
