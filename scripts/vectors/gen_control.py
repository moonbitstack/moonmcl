"""Generate the LQR and Riccati replay set from SciPy and python-control.

References: SciPy 1.17.1 `scipy.linalg.solve_discrete_are` for the stabilising
solution of the discrete algebraic Riccati equation, and python-control 0.10.x
`control.dlqr` for the gain. The two agree with each other, which is the point of
taking both: the cost matrix comes from one and the gain from the other.

Each case is a discrete plant, the state and input weights, the cost matrix the
equation solves to, and the gain that goes with it.

Run from the repository root:

    python scripts/vectors/gen_control.py
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import control
import numpy as np
import scipy.linalg as la

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "control", "lqr_vectors_test.mbt")


def flatten(matrix):
    return [float(value) for value in np.asarray(matrix).reshape(-1)]


def case(a, b, q, r):
    p = la.solve_discrete_are(a, b, q, r)
    gain, cost, _ = control.dlqr(a, b, q, r)
    # The two references have to agree on the cost matrix, or the case is not one
    # worth replaying.
    assert np.allclose(p, cost, atol=1e-10), "references disagree"
    return {
        "a": flatten(a),
        "b": flatten(b),
        "q": flatten(q),
        "r": flatten(r),
        "p": flatten(p),
        "k": flatten(gain),
        "n": a.shape[0],
        "m": b.shape[1],
    }


def main():
    rng = np.random.default_rng(20261112)
    cases = []

    # A double integrator at two sample rates.
    for dt in (0.1, 0.05):
        a = np.array([[1.0, dt], [0.0, 1.0]])
        b = np.array([[dt * dt / 2.0], [dt]])
        cases.append(case(a, b, np.diag([1.0, 0.1]), np.array([[0.5]])))

    # A damped oscillator.
    dt = 0.02
    a = np.array([[1.0, dt], [-4.0 * dt, 1.0 - 0.3 * dt]])
    b = np.array([[0.0], [dt]])
    cases.append(case(a, b, np.eye(2), np.array([[0.2]])))

    # A three-state plant with two inputs.
    dt = 0.1
    a = np.array(
        [
            [1.0, dt, 0.0],
            [0.0, 1.0, dt],
            [0.0, -2.0 * dt, 1.0 - 0.4 * dt],
        ]
    )
    b = np.array([[0.0, 0.0], [dt, 0.0], [0.0, dt]])
    cases.append(case(a, b, np.diag([2.0, 1.0, 0.5]), np.diag([1.0, 2.0])))

    # Random stable plants, kept well inside the unit circle so the equation has a
    # stabilising solution.
    while len(cases) < 12:
        n = int(rng.integers(2, 4))
        m = int(rng.integers(1, 3))
        a = rng.uniform(-0.6, 0.6, size=(n, n))
        radius = max(abs(np.linalg.eigvals(a)))
        if radius >= 0.95:
            continue
        b = rng.uniform(-1.0, 1.0, size=(n, m))
        if np.linalg.matrix_rank(b) < m:
            continue
        q = np.diag(rng.uniform(0.5, 3.0, size=n))
        r = np.diag(rng.uniform(0.2, 2.0, size=m))
        item = case(a, b, q, r)
        gain = np.array(item["k"]).reshape(m, n)
        closed = a - b @ gain
        if max(abs(np.linalg.eigvals(closed))) > 0.95:
            continue
        cases.append(item)

    blocks = []
    for item in cases:
        blocks.append(
            """  {
    a: %s,
    b: %s,
    q: %s,
    r: %s,
    cost: %s,
    gain: %s,
    rows: %d,
    columns: %d,
    inputs: %d,
  }"""
            % (
                moonbit_source.array(item["a"]),
                moonbit_source.array(item["b"]),
                moonbit_source.array(item["q"]),
                moonbit_source.array(item["r"]),
                moonbit_source.array(item["p"]),
                moonbit_source.array(item["k"]),
                item["n"],
                item["n"],
                item["m"],
            )
        )
    body = """
///|
/// One discrete Riccati problem: the plant, the weights, and the cost matrix and
/// gain the references solve it to.
struct LqrCase {
  a : Array[Double]
  b : Array[Double]
  q : Array[Double]
  r : Array[Double]
  cost : Array[Double]
  gain : Array[Double]
  rows : Int32
  columns : Int32
  inputs : Int32
}

///|
fn lqr_cases() -> Array[LqrCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_control.py",
        "SciPy 1.17.1 and python-control 0.10.2",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
