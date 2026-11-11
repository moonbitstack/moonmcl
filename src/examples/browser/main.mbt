// Entry point of the documentation build. It publishes one function on the
// page's global object; the page calls it with a request and reads the answer.
// The build exists only for the site, so it is restricted to the JS target.

///|
/// Hand `dispatch` to the page as `globalThis.MoonMCL`.
extern "js" fn js_publish(f : (String) -> String) = "(f) => { globalThis.MoonMCL = f; }"

///|
/// Route one request to its verb.
fn dispatch(request : String) -> String {
  let parts : Array[String] = request
    .split(";")
    .map(fn(v) { v.to_owned() })
    .collect()
  if parts.length() == 0 {
    return "ERR empty"
  }
  match parts[0] {
    "pid" =>
      pid_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field(parts, 4),
        field(parts, 5),
        field_index(parts, 6),
        field(parts, 7),
        field(parts, 8),
        field(parts, 9),
      )
    "scurve" =>
      scurve_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field(parts, 4),
        field_index(parts, 5),
      )
    "plan" =>
      plan_trace(
        field_index(parts, 1),
        field_index(parts, 2),
        index_list(parts[3]),
        field_index(parts, 4),
        field_index(parts, 5),
        field_index(parts, 6),
      )
    "kf" =>
      kf_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field_index(parts, 4),
      )
    "drive" =>
      drive_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field(parts, 4),
        field_index(parts, 5),
      )
    "lqr" =>
      lqr_trace(
        field(parts, 1),
        field(parts, 2),
        field_index(parts, 3),
        field(parts, 4),
        field(parts, 5),
      )
    "mpc" =>
      mpc_trace(
        field(parts, 1),
        field(parts, 2),
        field_index(parts, 3),
        field_index(parts, 4),
        field(parts, 5),
        field(parts, 6),
      )
    "follow" =>
      follow_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field_index(parts, 4),
        field(parts, 5),
      )
    "sync" =>
      sync_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field(parts, 4),
        field(parts, 5),
        field_index(parts, 6),
      )
    "space" =>
      space_trace(
        field(parts, 1),
        field(parts, 2),
        field(parts, 3),
        field(parts, 4),
        field(parts, 5),
        field(parts, 6),
        field(parts, 7),
        field_index(parts, 8),
        field_index(parts, 9),
      )
    "delta" =>
      delta_trace(field(parts, 1), field(parts, 2), field_index(parts, 3))
    _ => "ERR unknown verb"
  }
}

///|
fn main {
  js_publish(dispatch)
}
