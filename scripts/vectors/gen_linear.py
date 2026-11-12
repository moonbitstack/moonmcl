"""Generate the dense linear algebra replay set from NumPy and SciPy.

References: NumPy 2.4.6 `numpy.linalg` for solve, inverse, determinant,
Cholesky, QR and the symmetric eigenvalues, and SciPy 1.17.1
`scipy.linalg.expm` for the matrix exponential. Every matrix is fixed by a
seed, so the generated file is byte-for-byte reproducible.

A QR factorisation is unique only up to the sign of each column of Q and the
matching row of R. Both the reference and the library normalise that sign in
the same way before the vectors are written, and the test does the same again.

Run from the repository root:

    python scripts/vectors/gen_linear.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import scipy.linalg as la

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "..", "src", "linear", "linalg_vectors_test.mbt")


def flat(matrix):
    return [float(v) for v in np.asarray(matrix).reshape(-1)]


def normalised_qr(a):
    q, r = np.linalg.qr(a)
    for j in range(q.shape[1]):
        pivot = int(np.argmax(np.abs(q[:, j])))
        if q[pivot, j] < 0:
            q[:, j] *= -1.0
            r[j, :] *= -1.0
    return q, r


def main():
    rng = np.random.default_rng(20261112)
    cases = []
    for n in (2, 3, 4):
        for _ in range(3):
            a = rng.uniform(-1.5, 1.5, size=(n, n)) + np.eye(n) * 2.0
            if abs(np.linalg.det(a)) < 0.2:
                a += np.eye(n)
            b = rng.uniform(-2.0, 2.0, size=n)
            basis = rng.uniform(-1.0, 1.0, size=(n, n))
            spd = basis @ basis.T + np.eye(n) * n
            mixed = rng.uniform(-1.0, 1.0, size=(n, n))
            sym = 0.5 * (mixed + mixed.T)
            # Scaling and squaring keeps the Taylor series of `expm` in its
            # accurate range; a modest matrix checks both branches.
            small = a * (0.35 / max(1.0, np.abs(a).sum(axis=1).max()))
            q, r = normalised_qr(a)
            cases.append(
                {
                    "n": n,
                    "a": flat(a),
                    "b": flat(b),
                    "spd": flat(spd),
                    "sym": flat(sym),
                    "small": flat(small),
                    "solution": flat(np.linalg.solve(a, b)),
                    "inverse": flat(np.linalg.inv(a)),
                    "det": float(np.linalg.det(a)),
                    "cholesky": flat(np.linalg.cholesky(spd)),
                    "values": flat(np.linalg.eigh(sym)[0]),
                    "exponential": flat(la.expm(small)),
                    "q": flat(q),
                    "r": flat(r),
                }
            )

    blocks = []
    for c in cases:
        blocks.append(
            """  {
    a: %s,
    b: %s,
    spd: %s,
    sym: %s,
    small: %s,
    solution: %s,
    inverse: %s,
    determinant: %s,
    cholesky: %s,
    eigenvalues: %s,
    exponential: %s,
    q: %s,
    r: %s,
    n: %d,
  }"""
            % (
                moonbit_source.array(c["a"]),
                moonbit_source.array(c["b"]),
                moonbit_source.array(c["spd"]),
                moonbit_source.array(c["sym"]),
                moonbit_source.array(c["small"]),
                moonbit_source.array(c["solution"]),
                moonbit_source.array(c["inverse"]),
                moonbit_source.number(c["det"]),
                moonbit_source.array(c["cholesky"]),
                moonbit_source.array(c["values"]),
                moonbit_source.array(c["exponential"]),
                moonbit_source.array(c["q"]),
                moonbit_source.array(c["r"]),
                c["n"],
            )
        )
    body = """
///|
/// One matrix set and everything the references computed from it. Every matrix
/// is stored row by row.
struct LinalgCase {
  a : Array[Double]
  b : Array[Double]
  spd : Array[Double]
  sym : Array[Double]
  small : Array[Double]
  solution : Array[Double]
  inverse : Array[Double]
  determinant : Double
  cholesky : Array[Double]
  eigenvalues : Array[Double]
  exponential : Array[Double]
  q : Array[Double]
  r : Array[Double]
  n : Int32
}

///|
fn linalg_cases() -> Array[LinalgCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_linear.py",
        "NumPy 2.4.6 and SciPy 1.17.1",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
