// A second opinion on both car models. The primary tables replay `dubins` and
// rsplan; PythonRobotics implements the same two cars in files written separately
// from either, and the tables here are a fresh draw of poses, so agreement is two
// independent implementations agreeing with this library rather than one agreeing
// with itself. The comparison is on the length alone: the primary tables already
// pin the poses along the path and the cusp count, and the length is what both
// references solve for exactly.

///|
fn same_length(actual : Double, expected : Double) -> Bool {
  (actual - expected).abs() <= 1.0e-9 * (1.0 + expected.abs())
}

///|
test "dubins lengths match a second implementation" {
  let cases = dubins_cross_cases()
  assert_eq(cases.length() >= 24, true)
  for case in cases {
    let car = @planning.Dubins::new(turning_radius=case.radius) catch {
      _ => abort("the reference planned at a radius the car refused")
    }
    let start = @linear.Vec::of(case.start)
    let goal = @linear.Vec::of(case.goal)
    let path = car.path(start~, goal~) catch {
      _ => abort("the reference planned a pose the car refused")
    }
    assert_eq(same_length(path.length, case.length), true)
    // The path has to end where the reference's walk ends, which the generator
    // checked was the goal it was given.
    let end = path.at(path.length)
    assert_eq((end.get(0) - case.goal[0]).abs() < 1.0e-9, true)
    assert_eq((end.get(1) - case.goal[1]).abs() < 1.0e-9, true)
  }
}

///|
test "reeds shepp lengths match a second implementation" {
  let cases = reeds_shepp_cross_cases()
  assert_eq(cases.length() >= 24, true)
  for case in cases {
    let car = @planning.ReedsShepp::new(turning_radius=case.radius) catch {
      _ => abort("the reference planned at a radius the car refused")
    }
    let start = @linear.Vec::of(case.start)
    let goal = @linear.Vec::of(case.goal)
    let path = car.path(start~, goal~) catch {
      _ => abort("the reference planned a pose the car refused")
    }
    assert_eq(same_length(path.length, case.length), true)
    let end = path.at(path.length)
    assert_eq((end.get(0) - case.goal[0]).abs() < 1.0e-9, true)
    assert_eq((end.get(1) - case.goal[1]).abs() < 1.0e-9, true)
  }
}

///|
test "the reversing car is never longer than the forward-only one" {
  // Two references, one inequality that has to hold between them: the car that
  // may back up can always drive the forward-only path, so its shortest path is
  // at most as long. Both tables draw the same arena at the same radius range.
  for case in reeds_shepp_cross_cases() {
    let start = @linear.Vec::of(case.start)
    let goal = @linear.Vec::of(case.goal)
    let forward = @planning.Dubins::new(turning_radius=case.radius).path(
      start~,
      goal~,
    ) catch {
      _ => continue
    }
    let reversing = @planning.ReedsShepp::new(turning_radius=case.radius).path(
      start~,
      goal~,
    ) catch {
      _ => continue
    }
    assert_eq(reversing.length <= forward.length + 1.0e-9, true)
  }
}
