"""Generate the discretisation replay set from SciPy.

Reference: SciPy 1.18.1 `scipy.signal.cont2discrete`, which converts a continuous
state-space plant with the same three rules this library offers: `zoh` (the input
held over the period), `bilinear` (Tustin) and `euler` (forward Euler). Each case
is a continuous plant and a period; the expected values are the discrete `a` and
`b` the reference produces.

The observation matrices are the identity and zero throughout, because every rule
leaves `c` and `d` alone: only the state and input matrices are transformed.

Run from the repository root:

    python scripts/vectors/gen_discretize.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from scipy.signal import cont2discrete

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(
    HERE, "..", "..", "src", "contract", "discretize_vectors_test.mbt"
)


def flat(matrix):
    return [float(v) for v in np.asarray(matrix).reshape(-1)]


def pair(a, b, c, d, dt, method):
    ad, bd, _, _, _ = cont2discrete((a, b, c, d), dt, method=method)
    return flat(ad), flat(bd)


def main():
    rng = np.random.default_rng(20261112)
    cases = []
    # Plants whose poles are stable but not all equal: a double integrator, a
    # damped oscillator, and random matrices with a spectrum inside the left half
    # plane, so neither the exponential nor the bilinear transform is trivial.
    plants = [
        (np.array([[0.0, 1.0], [0.0, 0.0]]), np.array([[0.0], [1.0]])),
        (
            np.array([[0.0, 1.0], [-4.0, -0.6]]),
            np.array([[0.0], [1.0]]),
        ),
        (
            np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [-6.0, -11.0, -6.0]]),
            np.array([[0.0], [0.0], [1.0]]),
        ),
    ]
    for _ in range(3):
        n = int(rng.integers(2, 5))
        m = int(rng.integers(1, 3))
        a = rng.uniform(-2.0, 2.0, size=(n, n))
        # Pull every eigenvalue into the left half plane.
        values, vectors = np.linalg.eig(a)
        a = np.real(vectors @ np.diag(-np.abs(values) - 0.5) @ np.linalg.inv(vectors))
        b = rng.uniform(-1.5, 1.5, size=(n, m))
        plants.append((a, b))
    for a, b in plants:
        n, m = b.shape
        c = np.eye(n)
        d = np.zeros((n, m))
        for dt in (0.01, 0.1, 0.5):
            zoh_a, zoh_b = pair(a, b, c, d, dt, "zoh")
            tustin_a, tustin_b = pair(a, b, c, d, dt, "bilinear")
            euler_a, euler_b = pair(a, b, c, d, dt, "euler")
            cases.append(
                {
                    "n": n,
                    "m": m,
                    "dt": dt,
                    "a": flat(a),
                    "b": flat(b),
                    "zoh_a": zoh_a,
                    "zoh_b": zoh_b,
                    "tustin_a": tustin_a,
                    "tustin_b": tustin_b,
                    "euler_a": euler_a,
                    "euler_b": euler_b,
                }
            )

    blocks = []
    for c in cases:
        blocks.append(
            """  {
    a: %s,
    b: %s,
    dt: %s,
    zoh_a: %s,
    zoh_b: %s,
    tustin_a: %s,
    tustin_b: %s,
    euler_a: %s,
    euler_b: %s,
    n: %d,
    m: %d,
  },"""
            % (
                moonbit_source.array(c["a"]),
                moonbit_source.array(c["b"]),
                moonbit_source.number(c["dt"]),
                moonbit_source.array(c["zoh_a"]),
                moonbit_source.array(c["zoh_b"]),
                moonbit_source.array(c["tustin_a"]),
                moonbit_source.array(c["tustin_b"]),
                moonbit_source.array(c["euler_a"]),
                moonbit_source.array(c["euler_b"]),
                c["n"],
                c["m"],
            )
        )
    body = """
///|
/// One continuous plant, a period, and the discrete pair each rule produces. Both
/// matrices are stored row by row.
struct DiscretizeCase {
  a : Array[Double]
  b : Array[Double]
  dt : Double
  zoh_a : Array[Double]
  zoh_b : Array[Double]
  tustin_a : Array[Double]
  tustin_b : Array[Double]
  euler_a : Array[Double]
  euler_b : Array[Double]
  n : Int32
  m : Int32
}

///|
fn discretize_cases() -> Array[DiscretizeCase] {
  [
%s
  ]
}
""" % "\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_discretize.py",
        "scipy.signal.cont2discrete (SciPy 1.18.1)",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
