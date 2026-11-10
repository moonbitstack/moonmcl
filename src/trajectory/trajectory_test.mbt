// Tests for the trajectory package. The durations of the jerk-limited profile are
// the ones the reference implementation returns for the same limits and boundary
// conditions; the polynomial profiles are checked against the boundary states
// they promise, and the splines against the knots they pass through.

///|
fn refused(f : () -> Unit raise @linear.Refused) -> @linear.Refused? {
  try {
    f()
    None
  } catch {
    e => Some(e)
  }
}

///|
fn close(a : Double, b : Double, tolerance~ : Double = 1.0e-9) -> Bool {
  (a - b).abs() <= tolerance
}

// --- trapezoid ------------------------------------------------------------

///|
test "trapezoid reaches its distance and its limits" {
  let profile = @trajectory.trapezoid(distance=10.0, v_max=5.0, a_max=10.0)
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, 10.0, tolerance=1.0e-12), true)
  assert_eq(close(end.velocity, 0.0, tolerance=1.0e-12), true)
  // The peak is the velocity limit here, so the cruise is real.
  assert_eq(close(profile.peak_velocity, 5.0, tolerance=1.0e-12), true)
  let middle = profile.sample(profile.duration() / 2.0)
  assert_eq(close(middle.velocity, 5.0, tolerance=1.0e-12), true)
}

///|
test "trapezoid over a short distance never reaches the velocity limit" {
  let profile = @trajectory.trapezoid(distance=0.1, v_max=5.0, a_max=10.0)
  assert_eq(profile.peak_velocity < 5.0, true)
  assert_eq(
    close(profile.sample(profile.duration()).position, 0.1, tolerance=1.0e-12),
    true,
  )
  // A triangular velocity profile: half the distance is covered by the ramp.
  assert_eq(
    close(profile.duration(), 2.0 * (0.1 / 10.0).sqrt(), tolerance=1.0e-12),
    true,
  )
}

///|
test "trapezoid runs backwards for a negative distance" {
  let profile = @trajectory.trapezoid(distance=-3.0, v_max=2.0, a_max=4.0)
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, -3.0, tolerance=1.0e-12), true)
  assert_eq(end.velocity <= 0.0, true)
}

// --- S-curve --------------------------------------------------------------

///|
test "scurve duration matches the reference for a distance that cruises" {
  let profile = @trajectory.scurve(
    distance=10.0,
    v_max=5.0,
    a_max=10.0,
    j_max=30.0,
  )
  assert_eq(
    close(profile.duration(), 2.8333333333333335, tolerance=1.0e-12),
    true,
  )
  assert_eq(
    close(profile.sample(profile.duration()).position, 10.0, tolerance=1.0e-9),
    true,
  )
}

///|
test "scurve duration matches the reference for a distance that cannot cruise" {
  let profile = @trajectory.scurve(
    distance=1.0,
    v_max=5.0,
    a_max=10.0,
    j_max=30.0,
  )
  assert_eq(
    close(profile.duration(), 1.0217459098581222, tolerance=1.0e-9),
    true,
  )
  assert_eq(
    close(profile.sample(profile.duration()).position, 1.0, tolerance=1.0e-9),
    true,
  )
}

///|
test "scurve keeps the jerk inside its limit" {
  let profile = @trajectory.scurve(
    distance=1.0,
    v_max=5.0,
    a_max=10.0,
    j_max=30.0,
  )
  let steps = 200
  for i in 0..<=steps {
    let t = profile.duration() * i.to_double() / steps.to_double()
    let sample = profile.sample(t)
    assert_eq(sample.jerk.abs() <= 30.0 + 1.0e-9, true)
    assert_eq(sample.acceleration.abs() <= 10.0 + 1.0e-9, true)
  }
}

///|
test "scurve meets the boundary conditions it was given" {
  let profile = @trajectory.scurve(
    distance=10.0,
    v_max=5.0,
    a_max=10.0,
    j_max=30.0,
    start_velocity=1.0,
    start_acceleration=0.0,
    end_velocity=2.0,
    end_acceleration=0.0,
  )
  assert_eq(
    close(profile.duration(), 2.4830699929429825, tolerance=1.0e-9),
    true,
  )
  let start = profile.sample(0.0)
  assert_eq(close(start.velocity, 1.0, tolerance=1.0e-12), true)
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, 10.0, tolerance=1.0e-9), true)
  assert_eq(close(end.velocity, 2.0, tolerance=1.0e-9), true)
}

///|
test "scurve runs backwards for a negative distance" {
  let profile = @trajectory.scurve(
    distance=-10.0,
    v_max=5.0,
    a_max=10.0,
    j_max=30.0,
  )
  assert_eq(
    close(profile.duration(), 2.8333333333333335, tolerance=1.0e-12),
    true,
  )
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, -10.0, tolerance=1.0e-9), true)
}

///|
test "scurve refuses a boundary acceleration outside the limit" {
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @trajectory.scurve(
          distance=1.0,
          v_max=5.0,
          a_max=2.0,
          j_max=30.0,
          start_acceleration=5.0,
        ),
      )
    }) {
    Some(@linear.Refused::Unreachable) => ()
    _ => abort("expected Unreachable")
  }
}

///|
test "scurve refuses a boundary velocity against the direction of travel" {
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @trajectory.scurve(
          distance=1.0,
          v_max=5.0,
          a_max=10.0,
          j_max=30.0,
          start_velocity=-2.0,
        ),
      )
    }) {
    Some(@linear.Refused::Unreachable) => ()
    _ => abort("expected Unreachable")
  }
}

// --- polynomials ----------------------------------------------------------

///|
test "cubic matches both boundary states" {
  let profile = @trajectory.cubic(
    start=0.0,
    end=1.0,
    start_velocity=0.0,
    end_velocity=0.0,
    duration=2.0,
  )
  assert_eq(close(profile.sample(0.0).position, 0.0), true)
  assert_eq(close(profile.sample(0.0).velocity, 0.0), true)
  assert_eq(close(profile.sample(2.0).position, 1.0), true)
  assert_eq(close(profile.sample(2.0).velocity, 0.0), true)
  // A cubic with matching end velocities peaks at 1.5 times the average speed.
  assert_eq(close(profile.sample(1.0).velocity, 0.75), true)
}

///|
test "quintic matches both accelerations" {
  let profile = @trajectory.quintic(
    start=0.0,
    end=1.0,
    start_velocity=0.0,
    start_acceleration=0.0,
    end_velocity=0.0,
    end_acceleration=0.0,
    duration=1.0,
  )
  assert_eq(close(profile.sample(0.0).acceleration, 0.0), true)
  assert_eq(close(profile.sample(1.0).position, 1.0), true)
  assert_eq(close(profile.sample(1.0).acceleration, 0.0), true)
}

// --- splines --------------------------------------------------------------

///|
test "cubic spline passes through its knots" {
  let spline = @trajectory.cubic_spline(knots=[
    (0.0, 0.0),
    (1.0, 2.0),
    (2.0, 1.0),
    (3.0, 3.0),
  ])
  assert_eq(close(spline.sample(0.0).position, 0.0), true)
  assert_eq(close(spline.sample(1.0).position, 2.0), true)
  assert_eq(close(spline.sample(2.0).position, 1.0), true)
  assert_eq(close(spline.sample(3.0).position, 3.0), true)
  assert_eq(close(spline.duration(), 3.0), true)
}

///|
test "cubic spline has no curvature at its ends" {
  let spline = @trajectory.cubic_spline(knots=[
    (0.0, 0.0),
    (1.0, 2.0),
    (2.0, 1.0),
  ])
  assert_eq(close(spline.sample(0.0).acceleration, 0.0), true)
  assert_eq(close(spline.sample(2.0).acceleration, 0.0), true)
}

///|
test "b-spline interpolates the ends of its polygon" {
  let spline = @trajectory.b_spline(
    control=[0.0, 1.0, 2.0, 3.0],
    degree=3,
    duration=1.0,
  )
  assert_eq(close(spline.sample(0.0).position, 0.0), true)
  assert_eq(close(spline.sample(1.0).position, 3.0), true)
  // A cubic B-spline over a straight polygon is a straight line.
  assert_eq(close(spline.sample(0.5).position, 1.5, tolerance=1.0e-9), true)
}

// --- synchronised axes ----------------------------------------------------

///|
test "a synchronised set finishes every axis at once" {
  let short = @trajectory.trapezoid(distance=1.0, v_max=1.0, a_max=1.0)
  let long = @trajectory.trapezoid(distance=4.0, v_max=1.0, a_max=1.0)
  let set = @trajectory.Synced::new(axes=[short, long])
  assert_eq(close(set.duration(), long.duration()), true)
  let samples = set.sample_all(set.duration())
  assert_eq(close(samples[0].position, 1.0, tolerance=1.0e-9), true)
  assert_eq(close(samples[1].position, 4.0, tolerance=1.0e-9), true)
  // Halfway through, the short axis has covered half of its distance.
  let halfway = set.sample_all(set.duration() / 2.0)
  assert_eq(close(halfway[0].position, 0.5, tolerance=1.0e-9), true)
}

///|
test "scurve meets boundary accelerations and keeps its limits" {
  let profile = @trajectory.scurve(
    distance=2.0,
    v_max=3.0,
    a_max=4.0,
    j_max=20.0,
    start_velocity=1.0,
    start_acceleration=0.5,
  )
  let start = profile.sample(0.0)
  assert_eq(close(start.velocity, 1.0, tolerance=1.0e-12), true)
  assert_eq(close(start.acceleration, 0.5, tolerance=1.0e-12), true)
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, 2.0, tolerance=1.0e-6), true)
  assert_eq(close(end.velocity, 0.0, tolerance=1.0e-6), true)
  let steps = 400
  for i in 0..<=steps {
    let sample = profile.sample(
      profile.duration() * i.to_double() / steps.to_double(),
    )
    assert_eq(sample.jerk.abs() <= 20.0 + 1.0e-9, true)
    assert_eq(sample.acceleration.abs() <= 4.0 + 1.0e-6, true)
  }
  ignore(profile)
}
