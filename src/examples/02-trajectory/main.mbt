// A jerk-limited move, sampled: the velocity ramps instead of stepping, which is
// what keeps a real axis from ringing.
//
//     moon run src/examples/02-trajectory

///|
#coverage.skip
fn main raise {
  let profile = @mctl.scurve(distance=0.5, v_max=1.0, a_max=4.0, j_max=40.0)
  println("duration " + profile.duration().to_string())
  for step in 0..<=10 {
    let t = profile.duration() * step.to_double() / 10.0
    let sample = profile.sample(t)
    println(
      "t=" +
      t.to_string() +
      "  q=" +
      sample.position.to_string() +
      "  v=" +
      sample.velocity.to_string() +
      "  a=" +
      sample.acceleration.to_string(),
    )
  }
}
