// What the jerk-limited profile promises over the whole domain it accepts, and
// where it draws the line. The reference replay in `scurve_reference_test.mbt`
// pins the durations of the moves ruckig also solves; these tests sweep the
// boundary states instead, so a move that is accepted is checked against the
// conditions it was asked for rather than against a table.

///|
/// The limits the sweep is run at.
const V_MAX : Double = 5.0

///|
const A_MAX : Double = 2.0

///|
const J_MAX : Double = 20.0

///|
/// One boundary state and a distance to cover it over.
struct Sweep {
  distance : Double
  start_velocity : Double
  start_acceleration : Double
  end_velocity : Double
  end_acceleration : Double
}

///|
fn sweep_cases() -> Array[Sweep] {
  let distances = [0.05, 0.2, 1.0, 3.0, 8.0, 20.0]
  let velocities = [0.0, 0.5, 1.0]
  let accelerations = [-1.0, 0.0, 1.0]
  let cases = []
  for distance in distances {
    for start_velocity in velocities {
      for end_velocity in velocities {
        for start_acceleration in accelerations {
          for end_acceleration in accelerations {
            cases.push({
              distance,
              start_velocity,
              start_acceleration,
              end_velocity,
              end_acceleration,
            })
          }
        }
      }
    }
  }
  cases
}

///|
/// The profile for one sweep case, or `None` when the profile says the move is
/// not one it can make. Any other refusal is a shape error and fails loudly:
/// every boundary state in the sweep is inside the limits.
fn profile_for(case : Sweep) -> @trajectory.Scurve? {
  Some(
    @trajectory.scurve(
      distance=case.distance,
      v_max=V_MAX,
      a_max=A_MAX,
      j_max=J_MAX,
      start_velocity=case.start_velocity,
      start_acceleration=case.start_acceleration,
      end_velocity=case.end_velocity,
      end_acceleration=case.end_acceleration,
    ),
  ) catch {
    @linear.Refused::Unreachable => None
    _ => abort("a sweep case should not be refused for its limits")
  }
}

///|
test "an accepted move lands on the state it was given" {
  let mut accepted = 0
  for case in sweep_cases() {
    match profile_for(case) {
      None => ()
      Some(profile) => {
        accepted = accepted + 1
        let start = profile.sample(0.0)
        let end = profile.sample(profile.duration())
        assert_eq(close(start.velocity, case.start_velocity), true)
        assert_eq(close(start.acceleration, case.start_acceleration), true)
        assert_eq(close(end.position, case.distance, tolerance=1.0e-7), true)
        assert_eq(
          close(end.velocity, case.end_velocity, tolerance=1.0e-7),
          true,
        )
        assert_eq(
          close(end.acceleration, case.end_acceleration, tolerance=1.0e-7),
          true,
        )
      }
    }
  }
  // The sweep is meant to exercise the profile, not to be refused wholesale.
  assert_eq(accepted > sweep_cases().length() / 2, true)
}

///|
test "an accepted move keeps the limits" {
  for case in sweep_cases() {
    match profile_for(case) {
      None => ()
      Some(profile) => {
        let steps = 200
        for i in 0..<=steps {
          let sample = profile.sample(
            profile.duration() * i.to_double() / steps.to_double(),
          )
          assert_eq(sample.velocity.abs() <= V_MAX + 1.0e-9, true)
          assert_eq(sample.acceleration.abs() <= A_MAX + 1.0e-9, true)
          assert_eq(sample.jerk.abs() <= J_MAX + 1.0e-9, true)
        }
      }
    }
  }
}

///|
test "a longer move never takes less time" {
  // Over one direction, with the boundary states and the limits held fixed, the
  // shortest profile for a longer distance is at least as long as for a shorter
  // one: the extra distance has to be travelled, and nothing else changed.
  let distances = [0.1, 0.3, 0.7, 1.5, 4.0, 9.0, 25.0]
  for start_velocity in [0.0, 0.5, 1.0] {
    for end_velocity in [0.0, 0.5, 1.0] {
      let durations = []
      for distance in distances {
        durations.push(
          profile_for({
            distance,
            start_velocity,
            start_acceleration: 0.0,
            end_velocity,
            end_acceleration: 0.0,
          }).map(fn(profile) { profile.duration() }),
        )
      }
      for i in 1..<durations.length() {
        match (durations[i - 1], durations[i]) {
          (Some(short), Some(long)) => assert_eq(long >= short - 1.0e-9, true)
          _ => ()
        }
      }
    }
  }
}

///|
test "a move that would have to reverse is refused" {
  // From three metres per second to one over half a metre: the distance is
  // shorter than the braking it takes to come down to one, so a profile along
  // one direction does not exist. The profile that would make it dips through
  // zero velocity, which is a different structure, and this one says so instead
  // of returning something that overshoots.
  let refused = try {
    let _ = @trajectory.scurve(
      distance=0.5,
      v_max=5.0,
      a_max=1.0,
      j_max=4.0,
      start_velocity=3.0,
      end_velocity=1.0,
    )
    false
  } catch {
    @linear.Refused::Unreachable => true
    _ => false
  }
  assert_eq(refused, true)
  // The same train of states over a distance that leaves room to brake is a
  // move the profile does make.
  let profile = @trajectory.scurve(
    distance=6.0,
    v_max=5.0,
    a_max=1.0,
    j_max=4.0,
    start_velocity=3.0,
    end_velocity=1.0,
  )
  let end = profile.sample(profile.duration())
  assert_eq(close(end.position, 6.0, tolerance=1.0e-7), true)
  assert_eq(close(end.velocity, 1.0, tolerance=1.0e-7), true)
  // Six metres is more than the machine can cover at the velocity limit in two
  // seconds, so the move cannot be shorter than that.
  assert_eq(profile.duration() > 1.2, true)
}
