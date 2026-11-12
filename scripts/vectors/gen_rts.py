"""Generate the smoothing replay set from filterpy.

Reference: filterpy's `KalmanFilter.rts_smoother`, the Rauch-Tung-Striebel
backward pass over a recorded forward run. Each case is a transition, a process
covariance, the filtered means and covariances filterpy held after every update
of a measurement sequence, and the smoothed means and covariances it produces
from them.

The filtered estimates are stored as they came out of the reference, not
recomputed here: the point of the replay is the backward pass on its own, so
feeding it the reference's own record separates a smoother that disagrees from a
filter that does.

Run from the repository root:

    python scripts/vectors/gen_rts.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from filterpy.kalman import KalmanFilter

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "estimation", "rts_vectors_test.mbt")


def flat(matrix):
    return [float(v) for v in np.asarray(matrix).reshape(-1)]


def smooth(transition, process, observation, initial_state, initial_covariance,
           measurement_covariance, measurements):
    n = transition.shape[0]
    z_dim = observation.shape[0]
    kf = KalmanFilter(dim_x=n, dim_z=z_dim)
    kf.F = transition
    kf.H = observation
    kf.Q = process
    kf.R = measurement_covariance
    kf.x = np.array(initial_state, dtype=float)
    kf.P = np.array(initial_covariance, dtype=float)
    states, covariances = [], []
    for z in measurements:
        kf.predict()
        kf.update(np.array(z, dtype=float))
        states.append(kf.x.copy())
        covariances.append(kf.P.copy())
    smoothed_states, smoothed_covariances = kf.rts_smoother(
        np.array(states), np.array(covariances)
    )[:2]
    return (
        np.array(states),
        np.array(covariances),
        np.asarray(smoothed_states),
        np.asarray(smoothed_covariances),
    )


def main():
    rng = np.random.default_rng(20261112)
    plants = []

    # A constant-velocity cart observed in position only.
    plants.append(
        dict(
            transition=np.array([[1.0, 0.1], [0.0, 1.0]]),
            process=np.diag([0.01, 0.02]),
            observation=np.array([[1.0, 0.0]]),
            initial_state=[0.0, 1.0],
            initial_covariance=np.diag([2.0, 3.0]),
            measurement_covariance=np.array([[0.1]]),
            measurements=[[0.05], [0.24], [0.31], [0.52], [0.61], [0.83]],
        )
    )
    # Constant acceleration, observed in position and velocity.
    plants.append(
        dict(
            transition=np.array(
                [[1.0, 0.2, 0.02], [0.0, 1.0, 0.2], [0.0, 0.0, 1.0]]
            ),
            process=np.diag([0.002, 0.01, 0.05]),
            observation=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
            initial_state=[0.0, 0.0, 0.5],
            initial_covariance=np.eye(3) * 4.0,
            measurement_covariance=np.diag([0.2, 0.05]),
            measurements=[
                [0.02, 0.11],
                [0.09, 0.31],
                [0.21, 0.52],
                [0.38, 0.72],
                [0.63, 0.95],
            ],
        )
    )
    # A rotating pair, so the backward pass has to work with a transition that is
    # neither triangular nor diagonal.
    angle = 0.15
    plants.append(
        dict(
            transition=np.array(
                [
                    [np.cos(angle), -np.sin(angle)],
                    [np.sin(angle), np.cos(angle)],
                ]
            ),
            process=np.diag([0.005, 0.005]),
            observation=np.eye(2),
            initial_state=[1.0, 0.0],
            initial_covariance=np.diag([0.5, 0.5]),
            measurement_covariance=np.diag([0.05, 0.05]),
            measurements=[
                [0.96, 0.18],
                [0.88, 0.36],
                [0.79, 0.51],
                [0.68, 0.66],
            ],
        )
    )
    # A random stable plant with a dense process covariance.
    for _ in range(2):
        n = 2
        a = rng.uniform(-0.4, 0.4, size=(n, n))
        values, vectors = np.linalg.eig(a)
        a = np.real(
            vectors @ np.diag(0.6 * values / np.abs(values)) @ np.linalg.inv(vectors)
        )
        q = rng.uniform(-0.01, 0.01, size=(n, n))
        q = q @ q.T + np.eye(n) * 0.01
        steps = int(rng.integers(4, 7))
        measurements = [
            [float(rng.uniform(-0.5, 0.5))] for _ in range(steps)
        ]
        plants.append(
            dict(
                transition=a,
                process=q,
                observation=np.array([[1.0, 0.3]]),
                initial_state=[0.2, -0.4],
                initial_covariance=np.eye(2) * 1.5,
                measurement_covariance=np.array([[0.2]]),
                measurements=measurements,
            )
        )

    blocks = []
    for plant in plants:
        states, covariances, smoothed_states, smoothed_covariances = smooth(
            plant["transition"],
            plant["process"],
            plant["observation"],
            plant["initial_state"],
            plant["initial_covariance"],
            plant["measurement_covariance"],
            plant["measurements"],
        )
        blocks.append(
            """  {
    transition: %s,
    process: %s,
    states: [%s],
    covariances: [%s],
    smoothed_states: [%s],
    smoothed_covariances: [%s],
    n: %d,
  },"""
            % (
                moonbit_source.array(flat(plant["transition"])),
                moonbit_source.array(flat(plant["process"])),
                ", ".join(moonbit_source.array(flat(row)) for row in states),
                ", ".join(
                    moonbit_source.array(flat(cov)) for cov in covariances
                ),
                ", ".join(
                    moonbit_source.array(flat(row)) for row in smoothed_states
                ),
                ", ".join(
                    moonbit_source.array(flat(cov))
                    for cov in smoothed_covariances
                ),
                plant["transition"].shape[0],
            )
        )

    body = """
///|
/// One recorded forward run and the smoothed pass the reference made of it.
/// Every vector and matrix is stored row by row.
struct RtsCase {
  transition : Array[Double]
  process : Array[Double]
  states : Array[Array[Double]]
  covariances : Array[Array[Double]]
  smoothed_states : Array[Array[Double]]
  smoothed_covariances : Array[Array[Double]]
  n : Int32
}

///|
fn rts_cases() -> Array[RtsCase] {
  [
%s
  ]
}
""" % "\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_rts.py",
        "filterpy 2.0.0 `KalmanFilter.rts_smoother`",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(plants))


if __name__ == "__main__":
    main()
