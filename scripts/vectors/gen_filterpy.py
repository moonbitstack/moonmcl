"""Generate the estimation replay set from filterpy.

Reference: filterpy 1.4.5 (`pip install filterpy`) — the linear Kalman filter, the
extended one over a plant with drag, and the unscented one with the same sigma
point spread the library ships as its preset (alpha 1e-3, beta 2, kappa 0).

Each case is a prior, the covariances, and a measurement sequence; the expected
values are the mean and the covariance the reference holds after every update.

Run from the repository root:

    python scripts/vectors/gen_filterpy.py
"""

import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from filterpy.kalman import (
    ExtendedKalmanFilter,
    KalmanFilter,
    MerweScaledSigmaPoints,
    UnscentedKalmanFilter,
)

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "estimation", "filterpy_vectors_test.mbt")

DRAG = 0.1
NOISE = random.Random(20261112)


def step(state, u, dt):
    x, v = state
    return np.array([x + dt * v, v + dt * (u[0] - DRAG * v * abs(v))])


def step_jacobian(state, u, dt):
    _, v = state
    return np.array([[1.0, dt], [0.0, 1.0 - 2.0 * DRAG * dt * abs(v)]])


def observe(state):
    x, v = state
    return np.array([x + 0.05 * v * v])


def observe_jacobian(state):
    _, v = state
    return np.array([[1.0, 0.1 * v]])


def flatten(matrix):
    return [float(value) for value in np.asarray(matrix).reshape(-1)]


def linear_case(rng, steps):
    dt = 0.1
    q = rng.uniform(0.001, 0.05)
    r = rng.uniform(0.05, 1.0)
    f = np.array([[1.0, dt], [0.0, 1.0]])
    h = np.array([[1.0, 0.0]])
    x = np.array([rng.uniform(-1.0, 1.0), rng.uniform(-0.5, 0.5)])
    p = np.array([[0.5, 0.0], [0.0, 0.5]])
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.F = f
    kf.H = h
    kf.Q = np.eye(2) * q
    kf.R = np.array([[r]])
    kf.x = x.copy()
    kf.P = p.copy()
    truth = np.array([1.5, 0.4])
    measurements = []
    states = []
    covariances = []
    for _ in range(steps):
        kf.predict()
        truth = f @ truth
        z = float(truth[0] + NOISE.gauss(0.0, math.sqrt(r)))
        measurements.append(z)
        kf.update(np.array([z]))
        states.append(kf.x.copy())
        covariances.append(kf.P.copy())
    return {
        "dt": dt,
        "q": flatten(np.eye(2) * q),
        "r": float(r),
        "x0": flatten(x),
        "p0": flatten(p),
        "measurements": measurements,
        "states": [flatten(s) for s in states],
        "covariances": [flatten(c) for c in covariances],
    }


def nonlinear_case(rng, steps):
    dt = 0.1
    q = rng.uniform(0.0005, 0.02)
    r = rng.uniform(0.05, 0.5)
    x = np.array([rng.uniform(-0.5, 0.5), rng.uniform(0.5, 2.0)])
    p = np.array([[0.2, 0.0], [0.0, 0.2]])
    truth = np.array([0.3, 1.2])
    measurements = []
    for _ in range(steps):
        truth = step(truth, np.array([0.0]), dt)
        measurements.append(float(observe(truth)[0] + NOISE.gauss(0.0, math.sqrt(r))))

    # extended filter
    ekf = ExtendedKalmanFilter(dim_x=2, dim_z=1)
    ekf.x = x.copy()
    ekf.P = p.copy()
    ekf.Q = np.eye(2) * q
    ekf.R = np.array([[r]])
    # filterpy propagates the mean linearly through F by default; an extended
    # filter carries the mean through the plant itself and linearises only the
    # covariance, so the propagation is replaced here.
    def predict_through_plant(u=0):
        ekf.x = step(ekf.x, np.array([0.0]), dt)

    ekf.predict_x = predict_through_plant
    ekf_states = []
    ekf_covariances = []
    for z in measurements:
        ekf.F = step_jacobian(ekf.x, np.array([0.0]), dt)
        ekf.predict()
        ekf.update(np.array([z]), HJacobian=observe_jacobian, Hx=observe)
        ekf_states.append(ekf.x.copy())
        ekf_covariances.append(ekf.P.copy())

    # unscented filter
    points = MerweScaledSigmaPoints(n=2, alpha=1e-3, beta=2.0, kappa=0.0)
    ukf = UnscentedKalmanFilter(
        dim_x=2,
        dim_z=1,
        dt=dt,
        fx=lambda state, dt: step(state, np.array([0.0]), dt),
        hx=observe,
        points=points,
    )
    ukf.x = np.array(x, dtype=float)
    ukf.P = np.array(p, dtype=float)
    ukf.Q = np.eye(2) * q
    ukf.R = np.array([[r]])
    ukf_states = []
    ukf_covariances = []
    for z in measurements:
        ukf.predict()
        ukf.update(np.array([z]))
        ukf_states.append(ukf.x.copy())
        ukf_covariances.append(ukf.P.copy())

    common = {
        "dt": dt,
        "q": flatten(np.eye(2) * q),
        "r": float(r),
        "x0": flatten(x),
        "p0": flatten(p),
        "measurements": measurements,
    }
    return (
        dict(common, states=[flatten(s) for s in ekf_states],
             covariances=[flatten(c) for c in ekf_covariances]),
        dict(common, states=[flatten(s) for s in ukf_states],
             covariances=[flatten(c) for c in ukf_covariances]),
    )


def case_source(case):
    states = ",\n".join("      " + moonbit_source.array(s) for s in case["states"])
    covariances = ",\n".join(
        "      " + moonbit_source.array(c) for c in case["covariances"]
    )
    return """  {
    dt: %s,
    process: %s,
    measurement: %s,
    prior: %s,
    covariance: %s,
    measurements: %s,
    states: [
%s
    ],
    covariances: [
%s
    ],
  }""" % (
        moonbit_source.number(case["dt"]),
        moonbit_source.array(case["q"]),
        moonbit_source.number(case["r"]),
        moonbit_source.array(case["x0"]),
        moonbit_source.array(case["p0"]),
        moonbit_source.array(case["measurements"]),
        states,
        covariances,
    )


def main():
    rng = np.random.default_rng(20261112)
    linear = [linear_case(rng, 15) for _ in range(3)]
    pairs = [nonlinear_case(rng, 15) for _ in range(2)]
    extended = [pair[0] for pair in pairs]
    unscented = [pair[1] for pair in pairs]

    body = """
///|
/// One filter run: the prior, the noise, the measurements, and what the reference
/// filter held after each update.
struct FilterCase {
  dt : Double
  process : Array[Double]
  measurement : Double
  prior : Array[Double]
  covariance : Array[Double]
  measurements : Array[Double]
  states : Array[Array[Double]]
  covariances : Array[Array[Double]]
}

///|
fn filterpy_linear_cases() -> Array[FilterCase] {
  [
%s
  ]
}

///|
fn filterpy_extended_cases() -> Array[FilterCase] {
  [
%s
  ]
}

///|
fn filterpy_unscented_cases() -> Array[FilterCase] {
  [
%s
  ]
}
""" % (
        ",\n".join(case_source(c) for c in linear),
        ",\n".join(case_source(c) for c in extended),
        ",\n".join(case_source(c) for c in unscented),
    )
    text = moonbit_source.bundle(
        "scripts/vectors/gen_filterpy.py", "filterpy 1.4.5", body
    )
    moonbit_source.write(OUT, text)


if __name__ == "__main__":
    main()
