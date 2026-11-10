// The facade, package by package. A type re-exported here loses its
// constructors, so a caller that builds or matches one writes the package name
// (`@contract.Sample::{ .. }`); everything else is reachable as `@mctl.`-prefixed.

// --- linear ---------------------------------------------------------------

///|
pub using @linear {type Vec}

///|
pub using @linear {type Mat}

///|
pub using @linear {type Lu}

///|
pub using @linear {type Qr}

///|
pub using @linear {type Eigh}

///|
pub using @linear {type Refused}

///|
pub using @linear {
  solve,
  solve_vec,
  inv,
  det,
  lu,
  qr,
  cholesky,
  cholesky_solve,
  eigh,
  norm,
  expm,
}

// --- contracts ------------------------------------------------------------

///|
pub using @contract {type Sample}

///|
pub using @contract {type Budget}

///|
pub using @contract {type StateSpace}

///|
pub using @contract {type Dynamics}

///|
pub using @contract {type Discretize}

///|
pub using @contract {type Integrator}

///|
pub using @contract {jacobian_step, motion_samples, budget}
