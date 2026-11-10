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

// --- control --------------------------------------------------------------

///|
pub using @control {type Pid}

///|
pub using @control {type PidConfig}

///|
pub using @control {type AntiWindup}

///|
pub using @control {type Output}

///|
pub using @control {type Feedforward}

///|
pub using @control {type FeedforwardRunner}

///|
pub using @control {type Cascade}

///|
pub using @control {type Saturating}

///|
pub using @control {type RateLimited}

///|
pub using @control {type Riccati}

///|
pub using @control {type Lqr}

///|
pub using @control {type IntegralLqr}

///|
pub using @control {type IlqrConfig}

///|
pub using @control {type Mpc}

///|
pub using @control {type MpcConfig}

///|
pub using @control {type QpConfig}

///|
pub using @control {type Pursuit}

///|
pub using @control {type Stanley}

///|
pub using @control {type ComputedTorque}

///|
pub using @control {type Impedance}

///|
pub using @control {type CbfConfig}

///|
pub using @control {
  pid_config,
  pid_gains,
  output_unbounded,
  feedforward,
  dare,
  lqr,
  integral_lqr,
  ilqr_config,
  ilqr,
  mpc_config,
  qp_config,
  box_qp,
  pursuit,
  stanley,
  cbf_config,
  cbf_filter,
}

// --- estimation -----------------------------------------------------------

///|
pub using @estimation {type Kalman}

///|
pub using @estimation {type ExtendedKalman}

///|
pub using @estimation {type UnscentedKalman}

///|
pub using @estimation {type UnscentedConfig}

///|
pub using @estimation {type ParticleFilter}

///|
pub using @estimation {type Complementary}

///|
pub using @estimation {type AlphaBetaGamma}

///|
pub using @estimation {unscented_config, rts}

// --- trajectory -----------------------------------------------------------

///|
pub using @trajectory {type Trapezoid}

///|
pub using @trajectory {type Scurve}

///|
pub using @trajectory {type Polynomial}

///|
pub using @trajectory {type CubicSpline}

///|
pub using @trajectory {type BSpline}

///|
pub using @trajectory {type Synced}

///|
pub using @trajectory {
  trapezoid,
  scurve,
  cubic,
  quintic,
  cubic_spline,
  b_spline,
}

// --- planning -------------------------------------------------------------

///|
pub using @planning {
  type Rrt,
  type RrtStar,
  type RrtConnect,
  type InformedRrtStar,
  type Prm,
}

///|
pub using @planning {type Dubins, type DubinsPath, type DubinsWord}

///|
pub using @planning {type DynamicWindow, type PotentialField}

///|
pub using @planning {type GridSearch, type GridSpace}

///|
pub using @grid {type Grid}

///|
pub using @planning {dynamic_window, potential_field, grid_path}

// --- kinematics -----------------------------------------------------------

///|
pub using @kinematics {
  type Differential,
  type Bicycle,
  type Ackermann,
  type Mecanum,
}

///|
pub using @kinematics {
  type Omnidirectional,
  type PlanarArm,
  type Scara,
  type Delta,
  type Gantry,
}

// --- contracts as traits --------------------------------------------------

///|
pub using @contract {trait Model}

///|
pub using @contract {trait Controller}

///|
pub using @contract {trait Estimator}

///|
pub using @contract {trait Trajectory}

///|
pub using @contract {trait Kinematics}

///|
pub using @contract {trait Space}

///|
pub using @contract {trait Validity}

///|
pub using @contract {trait Planner}
