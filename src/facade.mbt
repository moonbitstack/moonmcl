// The root facade. It re-exports the public types and free functions of every
// module package so that callers can import the library once (the examples do
// this under the alias `mctl`) and address the whole surface as
// `@mctl.Pid`, `@mctl.Kf`, `@mctl.Grid`, `@mctl.Pose`, `@mctl.Matrix`, and so
// on. Method resolution follows the re-exported types, so `@mctl.Pid::pid(...)`
// and the subsequent `pid.update(...)` calls work as if the item came from the
// module package itself. Each module can still be imported on its own.

// --- linear algebra (the base every other module builds on) ---------------

///|
pub using @linear {type Matrix}

///|
pub using @linear {type Vector}

// --- classical + modern control -------------------------------------------

///|
pub using @control {type Feedforward}

///|
pub using @control {type IntegralLqr}

///|
pub using @control {type Lookahead}

///|
pub using @control {type Mpc}

///|
pub using @control {type Pid}

///|
pub using @control {type Scurve}

///|
pub using @control {type Steering}

///|
pub using @control {dare}

///|
pub using @control {integral_lqr}

///|
pub using @control {lqr_gain}

///|
pub using @control {pure_pursuit}

///|
pub using @control {scurve_plan}

///|
pub using @control {stanley}

// --- state estimation ------------------------------------------------------

///|
pub using @estimation {type EKF}

///|
pub using @estimation {type Kf}

// --- grid path planning ----------------------------------------------------

///|
pub using @planning {type Grid}

///|
pub using @planning {type MinHeap}

///|
pub using @planning {astar}

///|
pub using @planning {dijkstra}

// --- differential-drive kinematics -----------------------------------------

///|
pub using @kinematics {type Pose}

///|
pub using @kinematics {type Twist}

///|
pub using @kinematics {type WheelVel}

///|
pub using @kinematics {forward_kinematics}

///|
pub using @kinematics {inverse_kinematics}

///|
pub using @kinematics {update_pose}
