"""Generate the numerical-Jacobian replay set from SciPy's complex step.

The `Model` and `Kinematics` contracts give their users a central-difference
Jacobian by default. SciPy 1.18.1 `scipy.optimize._numdiff.approx_derivative`
with `method="cs"` computes the same derivatives by complex step, which is
accurate to machine precision and shares no arithmetic with our scheme. Each
case is a nonlinear plant at a fixed state, input and sample time.

Run from the repository root:

    python scripts/vectors/gen_contract.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
from scipy.optimize._numdiff import approx_derivative

import moonbit_source

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(
    HERE, "..", "..", "src", "contract", "jacobian_vectors_test.mbt"
)


def step(x, u, dt):
    """A two-state nonlinear plant; every operation accepts complex arrays."""
    return np.array(
        [
            x[0] + dt * (x[1] + u[0] * np.sin(x[0])),
            x[1] + dt * (-0.8 * x[0] * x[1] + u[0] * np.cos(x[1])),
        ],
        dtype=x.dtype if hasattr(x, "dtype") else None,
    )


def observe(x):
    return np.array(
        [
            np.sqrt(x[0] * x[0] + x[1] * x[1] + 1.0),
            np.exp(-0.1 * x[0]) * np.sin(x[1]),
        ]
    )


def flat(matrix):
    return [float(v) for v in np.asarray(matrix).reshape(-1)]


def main():
    rng = np.random.default_rng(20261112)
    cases = []
    for _ in range(9):
        x = rng.uniform(-1.2, 1.2, size=2)
        u = rng.uniform(-0.8, 0.8, size=1)
        dt = float(rng.uniform(0.02, 0.2))
        jacobian_step = approx_derivative(
            lambda y: step(y, u, dt),
            x,
            method="cs",
            rel_step=1.0e-20,
        )
        jacobian_observe = approx_derivative(
            observe, x, method="cs", rel_step=1.0e-20
        )
        cases.append(
            {
                "x": flat(x),
                "u": flat(u),
                "dt": dt,
                "step_jacobian": flat(jacobian_step),
                "observe_jacobian": flat(jacobian_observe),
            }
        )

    blocks = []
    for c in cases:
        blocks.append(
            """  {
    x: %s,
    u: %s,
    dt: %s,
    step_jacobian: %s,
    observe_jacobian: %s,
  }"""
            % (
                moonbit_source.array(c["x"]),
                moonbit_source.array(c["u"]),
                moonbit_source.number(c["dt"]),
                moonbit_source.array(c["step_jacobian"]),
                moonbit_source.array(c["observe_jacobian"]),
            )
        )
    body = """
///|
/// One plant state, the input it is held at, the sample time, and the two
/// Jacobians the reference computed there.
struct JacobianCase {
  x : Array[Double]
  u : Array[Double]
  dt : Double
  step_jacobian : Array[Double]
  observe_jacobian : Array[Double]
}

///|
fn jacobian_cases() -> Array[JacobianCase] {
  [
%s
  ]
}
""" % ",\n".join(blocks)
    text = moonbit_source.bundle(
        "scripts/vectors/gen_contract.py",
        "SciPy 1.18.1 complex-step derivative",
        body,
    )
    moonbit_source.write(OUT, text)
    print("%d cases" % len(cases))


if __name__ == "__main__":
    main()
