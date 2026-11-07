// A jerk-limited (S-curve) single-axis trajectory profile. `scurve_plan`
// builds the minimal-time rest-to-rest profile from the origin to a target
// distance; `at` returns the exact position / velocity / acceleration / jerk at
// any instant by integrating the stored piecewise-constant jerk schedule. All
// planner distances are computed with that same integrator, so the schedule and
// the kinematics can never disagree.

///|
/// A single axis of a jerk-limited profile.
pub struct Scurve {
  // Piecewise-constant jerk schedule as (duration, jerk) pairs.
  segments : Array[(Double, Double)]
  // Signed total travel (negative for a reverse-direction profile).
  distance : Double
  // Total time of the profile.
  total_time : Double
}

///|
/// The (signed) position, velocity, acceleration and jerk at time `t`,
/// clamped to the profile's endpoints.
pub fn Scurve::at(
  self : Scurve,
  t : Double,
) -> (Double, Double, Double, Double) {
  let dir : Double = if self.distance >= 0.0 { 1.0 } else { -1.0 }
  let tt = if t < 0.0 {
    0.0
  } else if t > self.total_time {
    self.total_time
  } else {
    t
  }
  let (smag, vmag, amag, jmag) = integrate_segments(self.segments, tt)
  (dir * smag, dir * vmag, dir * amag, dir * jmag)
}

///|
fn integrate_segments(
  segments : Array[(Double, Double)],
  t : Double,
) -> (Double, Double, Double, Double) {
  let mut s = 0.0
  let mut v = 0.0
  let mut a = 0.0
  let mut jk = 0.0
  let mut remaining = t
  for (dur, j) in segments.iter() {
    jk = j
    let step = if remaining <= dur { remaining } else { dur }
    s += v * step + a * step * step / 2.0 + j * step * step * step / 6.0
    v += a * step + j * step * step / 2.0
    a += j * step
    remaining -= dur
    if remaining <= 0.0 {
      break
    }
  }
  (s, v, a, jk)
}

///|
/// The distance covered by the accelerating half `(tj, +j), (ta, 0), (tj, -j)`.
fn half_distance(tj : Double, ta : Double, j : Double) -> Double {
  let half : Array[(Double, Double)] = [(tj, j), (ta, 0.0), (tj, -j)]
  let (sd, _, _, _) = integrate_segments(half, 2.0 * tj + ta)
  sd
}

///|
/// The distance covered by the accelerating half of the pure-jerk triangle
/// `(t1, +j), (t1, -j)`.
fn half_distance_tri(t1 : Double, j : Double) -> Double {
  let half : Array[(Double, Double)] = [(t1, j), (t1, -j)]
  let (sd, _, _, _) = integrate_segments(half, 2.0 * t1)
  sd
}

///|
/// The full distance of the 4-segment pure-jerk triangle
/// `(t, +j), (t, -j), (t, -j), (t, +j)`, which is rest-to-rest.
fn triangle_distance(t : Double, j : Double) -> Double {
  let segs : Array[(Double, Double)] = [(t, j), (t, -j), (t, -j), (t, j)]
  let (sd, _, _, _) = integrate_segments(segs, 4.0 * t)
  sd
}

///|
/// Plan the minimal-time jerk-limited rest-to-rest profile from the origin to
/// `distance` (which may be negative), respecting `v_max`, `a_max` and `j_max`.
pub fn scurve_plan(
  distance : Double,
  v_max : Double,
  a_max : Double,
  j_max : Double,
) -> Scurve {
  guard v_max > 0.0 && a_max > 0.0 && j_max > 0.0 else {
    abort("scurve_plan: v_max, a_max and j_max must all be positive")
  }
  let d = distance.abs()
  if d == 0.0 {
    { segments: [], distance, total_time: 0.0, }
  } else {
    let (segments, total_time) = plan_segments(d, v_max, a_max, j_max)
    { segments, distance, total_time, }
  }
}

///|
fn plan_segments(
  d : Double,
  v : Double,
  a : Double,
  j : Double,
) -> (Array[(Double, Double)], Double) {
  let tj = a / j
  // The longest constant-acceleration phase that keeps the peak velocity at
  // v; when it is negative the velocity limit is reached during the jerk ramp
  // and the acceleration never reaches `a` (the pure-jerk triangle regime).
  let ta_max = v / a - tj
  if ta_max >= 0.0 {
    // Acceleration-limited regime.
    let s_half_max = half_distance(tj, ta_max, j)
    if d >= 2.0 * s_half_max {
      // 7-phase: peak velocity is v, fill the middle with constant velocity.
      let tc = (d - 2.0 * s_half_max) / v
      let segs : Array[(Double, Double)] = [
        (tj, j),
        (ta_max, 0.0),
        (tj, -j),
        (tc, 0.0),
        (tj, -j),
        (ta_max, 0.0),
        (tj, j),
      ]
      return (segs, 4.0 * tj + 2.0 * ta_max + tc)
    }
    // 5-phase: no constant-velocity phase; find ta in [0, ta_max] with
    // 2 * half_distance(ta) = d.
    let mut lo = 0.0
    let mut hi = ta_max
    let mut ta = ta_max
    for _ in 0..<100 {
      let mid = (lo + hi) / 2.0
      if 2.0 * half_distance(tj, mid, j) < d {
        lo = mid
      } else {
        hi = mid
      }
      ta = mid
    }
    let segs : Array[(Double, Double)] = [
      (tj, j),
      (ta, 0.0),
      (tj, -j),
      (tj, -j),
      (ta, 0.0),
      (tj, j),
    ]
    (segs, 2.0 * (tj + ta) + 2.0 * tj)
  } else {
    // Pure-jerk triangle regime: peak velocity v is reached during the jerk,
    // so the peak acceleration is below `a`. t1 = sqrt(v / j) gives peak
    // velocity j * t1^2 = v.
    let t1 = (v / j).sqrt()
    let s_half_tri = half_distance_tri(t1, j)
    if d >= 2.0 * s_half_tri {
      let tc = (d - 2.0 * s_half_tri) / v
      let segs : Array[(Double, Double)] = [
        (t1, j),
        (t1, -j),
        (tc, 0.0),
        (t1, -j),
        (t1, j),
      ]
      return (segs, 4.0 * t1 + tc)
    }
    // 3-phase triangle: no constant velocity; find t with triangle_distance = d.
    let mut lo = 1.0e-9
    let mut hi = 2.0 * t1
    let mut t = t1
    for _ in 0..<100 {
      let mid = (lo + hi) / 2.0
      if triangle_distance(mid, j) < d {
        lo = mid
      } else {
        hi = mid
      }
      t = mid
    }
    let segs : Array[(Double, Double)] = [(t, j), (t, -j), (t, -j), (t, j)]
    (segs, 4.0 * t)
  }
}
