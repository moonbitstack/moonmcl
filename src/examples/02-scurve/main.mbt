// A jerk-limited (S-curve) move: plan a rest-to-rest profile and print its
// time, peak speed and the position reached.
//
//     moon run src/examples/02-scurve

///|
#coverage.skip
fn main {
  // Move 100 units at most 5 units/s, 2 units/s^2, 1 unit/s^3.
  let profile = @mctl.scurve_plan(100.0, 5.0, 2.0, 1.0)
  println("total time  = " + profile.total_time.to_string())
  let (vmax, amax) = sample_max(profile, 800)
  println("peak speed  = " + vmax.to_string() + " (limit 5)")
  println("peak accel  = " + amax.to_string() + " (limit 2)")
  let (s_end, v_end, a_end, _) = profile.at(profile.total_time)
  println(
    "end         : s=" +
    s_end.to_string() +
    " v=" +
    v_end.to_string() +
    " a=" +
    a_end.to_string(),
  )
}

///|
fn sample_max(profile : @mctl.Scurve, steps : Int32) -> (Double, Double) {
  let dt = profile.total_time / steps.to_double()
  let mut vmax = 0.0
  let mut amax = 0.0
  let mut t = 0.0
  while t <= profile.total_time + 1.0e-12 {
    let (_, v, a, _) = profile.at(t)
    if v.abs() > vmax {
      vmax = v.abs()
    }
    if a.abs() > amax {
      amax = a.abs()
    }
    t = t + dt
  }
  (vmax, amax)
}
