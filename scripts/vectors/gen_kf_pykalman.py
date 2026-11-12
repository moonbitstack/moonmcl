"""Generate a second opinion on the linear filter and the smoother, from pykalman.

filterpy is the primary reference for both. pykalman runs the same two recursions
with its own code, so a fresh table is an independent check rather than a repeat.

For each case the table holds the plant, the noise covariances, the prior, the
measurement sequence, and the reference's filtered and smoothed means and
covariances. The smoothing figures come from the reference's own forward pass, the
way the primary table does it, so a disagreement there is the backward pass.

Run from the repository root:

    python scripts/vectors/gen_kf_pykalman.py
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
from pykalman import KalmanFilter

import moonbit_source

OUT = os.path.join(HERE, "..", "..", "src", "estimation", "kf_pykalman_vectors_test.mbt")


def flat(matrix):
    return [float(v) for v in np.asarray(matrix).reshape(-1)]


def main():
    plants = [
        dict(
            transition=np.array([[1.0, 0.2], [0.0, 1.0]]),
            process=np.diag([0.02, 0.05]),
            observation=np.array([[1.0, 0.0]]),
            prior=[0.0, 1.0],
            covariance=np.diag([2.0, 2.0]),
            noise=np.array([[0.12]]),
            measurements=[[0.06], [0.31], [0.52], [0.79], [1.02]],
        ),
        dict(
            transition=np.array(
                [[1.0, 0.1, 0.005], [0.0, 1.0, 0.1], [0.0, 0.0, 1.0]]
            ),
            process=np.diag([0.001, 0.01, 0.04]),
            observation=np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
            prior=[0.0, 0.0, 0.4],
            covariance=np.eye(3) * 3.0,
            noise=np.diag([0.15, 0.03]),
            measurements=[
                [0.03, 0.38],
                [0.10, 0.44],
                [0.19, 0.51],
                [0.31, 0.58],
                [0.44, 0.66],
            ],
        ),
        dict(
            transition=np.array([[0.95]]),
            process=np.array([[0.01]]),
            observation=np.array([[1.0]]),
            prior=[1.0],
            covariance=np.array([[1.0]]),
            noise=np.array([[0.25]]),
            measurements=[[0.92], [0.85], [0.71], [0.66]],
        ),
    ]

    cases = []
    for plant in plants:
        kf = KalmanFilter(
            transition_matrices=plant["transition"],
            observation_matrices=plant["observation"],
            transition_covariance=plant["process"],
            observation_covariance=plant["noise"],
            initial_state_mean=np.array(plant["prior"], dtype=float),
            initial_state_covariance=np.array(plant["covariance"], dtype=float),
            n_dim_state=plant["transition"].shape[0],
            n_dim_obs=plant["observation"].shape[0],
        )
        z = np.array(plant["measurements"])
        filtered_means, filtered_covs = kf.filter(z)
        smoothed_means, smoothed_covs = kf.smooth(z)
        assert len(filtered_means) == len(plant["measurements"])
        assert len(smoothed_means) == len(plant["measurements"])
        cases.append(
            {
                "transition": flat(plant["transition"]),
                "process": flat(plant["process"]),
                "observation": flat(plant["observation"]),
                "noise": flat(plant["noise"]),
                "prior": list(plant["prior"]),
                "covariance": flat(plant["covariance"]),
                "measurements": [row for row in plant["measurements"]],
                "filtered": [flat(row) for row in filtered_means],
                "filtered_covariance": [flat(c) for c in filtered_covs],
                "smoothed": [flat(row) for row in smoothed_means],
                "smoothed_covariance": [flat(c) for c in smoothed_covs],
                "n": plant["transition"].shape[0],
                "m": plant["observation"].shape[0],
            }
        )

    blocks = []
    for case in cases:
        blocks.append(
            """  {
    transition: %s,
    process: %s,
    observation: %s,
    noise: %s,
    prior: %s,
    covariance: %s,
    measurements: [%s],
    filtered: [%s],
    filtered_covariance: [%s],
    smoothed: [%s],
    smoothed_covariance: [%s],
    n: %d,
    m: %d,
  },"""
            % (
                moonbit_source.array(case["transition"]),
                moonbit_source.array(case["process"]),
                moonbit_source.array(case["observation"]),
                moonbit_source.array(case["noise"]),
                moonbit_source.array(case["prior"]),
                moonbit_source.array(case["covariance"]),
                ", ".join(moonbit_source.array(r) for r in case["measurements"]),
                ", ".join(moonbit_source.array(r) for r in case["filtered"]),
                ", ".join(moonbit_source.array(r) for r in case["filtered_covariance"]),
                ", ".join(moonbit_source.array(r) for r in case["smoothed"]),
                ", ".join(moonbit_source.array(r) for r in case["smoothed_covariance"]),
                case["n"],
                case["m"],
            )
        )
    body = """
///|
/// One run of a linear plant through the reference's filter and smoother. Every
/// vector and matrix is stored row by row.
struct KfPykalmanCase {
  transition : Array[Double]
  process : Array[Double]
  observation : Array[Double]
  noise : Array[Double]
  prior : Array[Double]
  covariance : Array[Double]
  measurements : Array[Array[Double]]
  filtered : Array[Array[Double]]
  filtered_covariance : Array[Array[Double]]
  smoothed : Array[Array[Double]]
  smoothed_covariance : Array[Array[Double]]
  n : Int32
  m : Int32
}

///|
fn kf_pykalman_cases() -> Array[KfPykalmanCase] {
  [
%s
  ]
}
""" % "\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_kf_pykalman.py",
        "pykalman 0.11.2 `KalmanFilter`",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
