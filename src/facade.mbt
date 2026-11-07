// The root facade. Re-exports the linear-algebra primitives so that callers
// can write `@moonmcl.Vector` and `@moonmcl.Matrix`; the per-module packages
// (`@moonmcl.control`, `@moonmcl.estimation`, ...) are imported as needed.

///|
pub using @linear {type Vector}

///|
pub using @linear {type Matrix}
