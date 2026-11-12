"""Generate a second opinion on the unscented filter, from pykalman.

The primary table for the filters comes from filterpy. pykalman implements the
same unscented recursion separately, so a second table over the same plant with a
fresh measurement sequence is an independent check rather than a repeat.

The plant is the cart with quadratic drag the primary table also uses; the noise
covariances, the prior and the measurements are its own. Only the mean after each
update is recorded: the covariance carries the same information and the filterpy
table already pins it.

Run from the repository root:

    python scripts/vectors/gen_ukf_pykalman.py
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import numpy as np
from pykalman import UnscentedKalmanFilter

import moonbit_source

OUT = os.path.join(HERE, "..", "..", "src", "estimation", "ukf_pykalman_vectors_test.mbt")

DRAG = 0.1


def step(state, noise, dt):
    """The cart: quadratic drag on the velocity, and the noise added to the state."""
    position, velocity = state
    return np.array([
        position + dt * velocity + noise[0],
        velocity + dt * (-DRAG * velocity * abs(velocity)) + noise[1],
    ])


def observe(state, noise):
    """A quadratic position sensor."""
    position, velocity = state
    return np.array([position + 0.05 * velocity * velocity + noise[0]])


def run(prior, covariance, process, measurement, measurements, dt):
    ukf = UnscentedKalmanFilter(
        transition_functions=lambda state, noise: step(state, noise, dt),
        observation_functions=lambda state, noise: observe(state, noise),
        transition_covariance=process,
        observation_covariance=measurement,
        initial_state_mean=prior,
        initial_state_covariance=covariance,
        n_dim_state=2,
        n_dim_obs=1,
    )
    means, _ = ukf.filter(np.array(measurements))
    # One row per measurement, the estimate after that update.
    return [list(map(float, mean)) for mean in means]


def main():
    dt = 0.5
    process = np.diag([0.01, 0.02])
    measurement = np.array([[0.1]])
    cases = []
    priors = [
        ([0.0, 0.0], [1.0, 0.0, 0.0, 1.0]),
        ([0.5, -0.4], [2.0, 0.1, 0.1, 1.5]),
        ([-0.3, 0.8], [0.5, 0.0, 0.0, 0.5]),
    ]
    sequences = [
        [0.05, 0.32, 0.61, 0.94, 1.24, 1.68],
        [0.60, 0.42, 0.71, 0.88, 1.05],
        [-0.10, -0.02, 0.24, 0.51, 0.79, 1.12, 1.40],
    ]
    for (prior, covariance), measurements in zip(priors, sequences):
        states = run(
            np.array(prior, dtype=float),
            np.array(covariance, dtype=float).reshape(2, 2),
            process,
            measurement,
            [[v] for v in measurements],
            dt,
        )
        assert len(states) == len(measurements)
        cases.append(
            {
                "prior": list(prior),
                "covariance": list(covariance),
                "measurements": measurements,
                "states": states,
                "dt": dt,
            }
        )

    blocks = []
    for case in cases:
        blocks.append(
            """  {
    prior: %s,
    covariance: %s,
    measurements: %s,
    states: [%s],
    dt: %s,
  },"""
            % (
                moonbit_source.array(case["prior"]),
                moonbit_source.array(case["covariance"]),
                moonbit_source.array(case["measurements"]),
                ", ".join(moonbit_source.array(s) for s in case["states"]),
                moonbit_source.number(case["dt"]),
            )
        )
    body = """
///|
/// One run of the cart through the reference's unscented filter. The states are
/// the means after each update; entry zero of the reference's own output is the
/// prior and is not stored here.
struct PykalmanCase {
  prior : Array[Double]
  covariance : Array[Double]
  measurements : Array[Double]
  states : Array[Array[Double]]
  dt : Double
}

///|
fn pykalman_cases() -> Array[PykalmanCase] {
  [
%s
  ]
}
""" % "\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_ukf_pykalman.py",
        "pykalman 0.11.2 `UnscentedKalmanFilter`",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
