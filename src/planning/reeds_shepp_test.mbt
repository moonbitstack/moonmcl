// The Reeds-Shepp car: the words themselves, the poses they reach, and the
// space that a sampling planner plans in. The shortest lengths are replayed
// against a reference implementation by `scripts/vectors/gen_reeds_shepp.py`,
// which writes the table this file reads.

///|
/// The heading difference folded into (-pi, pi].
fn angle_gap(a : Double, b : Double) -> Double {
  @math.atan2(@math.sin(a - b), @math.cos(a - b))
}

///|
test "reeds shepp path ends on the pose it was asked for" {
  let car = @planning.ReedsShepp::new(turning_radius=1.5)
  let poses = [
    ([0.0, 0.0, 0.0], [3.0, 1.0, 0.6]),
    ([0.0, 0.0, 0.0], [-3.0, 1.0, 0.6]),
    ([1.0, 2.0, -0.4], [1.4, 2.2, 2.0]),
    ([0.0, 0.0, 3.0], [0.2, -0.1, -3.0]),
  ]
  for (from, to) in poses {
    let start = @linear.Vec::of(from)
    let goal = @linear.Vec::of(to)
    let path = car.path(start~, goal~)
    let end = path.at(path.length)
    assert_eq(close(end.get(0), to[0], tolerance=1.0e-9), true)
    assert_eq(close(end.get(1), to[1], tolerance=1.0e-9), true)
    assert_eq(angle_gap(end.get(2), to[2]).abs() <= 1.0e-9, true)
  }
}

///|
test "a goal behind the car is reached by backing up" {
  let car = @planning.ReedsShepp::new(turning_radius=1.0)
  let start = @linear.Vec::of([0.0, 0.0, 0.0])
  let goal = @linear.Vec::of([-2.0, 0.0, 0.0])
  let path = car.path(start~, goal~)
  assert_eq(path.cusps() >= 0, true)
  assert_eq(close(path.length, 2.0, tolerance=1.0e-9), true)
  // The straight word alone reaches it; every piece runs backwards.
  let mut forward = false
  for segment in path.segments {
    if segment.forward > 0.0 && segment.length > 1.0e-12 {
      forward = true
    }
  }
  assert_eq(forward, false)
}

///|
test "all forty-eight words are reachable and land on the goal" {
  let car = @planning.ReedsShepp::new(turning_radius=1.0)
  let start = @linear.Vec::of([0.0, 0.0, 0.0])
  let seen = Array::make(48, 0)
  let mut seed : Int32 = 12345
  for _ in 0..<600 {
    seed = (seed * 1103515245 + 12345) % 2147483647
    let x = (seed % 80001).to_double() / 10000.0 - 4.0
    seed = (seed * 1103515245 + 12345) % 2147483647
    let y = (seed % 80001).to_double() / 10000.0 - 4.0
    seed = (seed * 1103515245 + 12345) % 2147483647
    let phi = (seed % 62832).to_double() / 10000.0 - 3.1416
    let goal = @linear.Vec::of([x, y, phi])
    for path in car.candidates(start~, goal~) {
      seen[path.word.code()] = seen[path.word.code()] + 1
      let end = path.at(path.length)
      assert_eq(close(end.get(0), x, tolerance=1.0e-6), true)
      assert_eq(close(end.get(1), y, tolerance=1.0e-6), true)
      assert_eq(angle_gap(end.get(2), phi).abs() <= 1.0e-6, true)
    }
  }
  for code in 0..<48 {
    assert_eq(seen[code] > 0, true)
  }
}

///|
test "the shortest candidate is the one the path returns" {
  let car = @planning.ReedsShepp::new(turning_radius=0.8)
  let start = @linear.Vec::of([0.0, 0.0, 0.0])
  let goal = @linear.Vec::of([2.0, 1.5, -1.0])
  let path = car.path(start~, goal~)
  for other in car.candidates(start~, goal~) {
    assert_eq(path.length <= other.length + 1.0e-12, true)
  }
}

///|
test "reversing beats a forward-only car on a pose behind it" {
  let car = @planning.ReedsShepp::new(turning_radius=1.0)
  let dubins = @planning.Dubins::new(turning_radius=1.0)
  let start = @linear.Vec::of([0.0, 0.0, 0.0])
  let goal = @linear.Vec::of([-1.0, 0.4, 0.0])
  let with_reverse = car.path(start~, goal~).length
  let forward_only = dubins.path(start~, goal~).length
  assert_eq(with_reverse < forward_only, true)
}

///|
test "the reeds shepp space joins two states along a path that reverses" {
  let car = @planning.ReedsShepp::new(turning_radius=1.0, extent=2.0)
  let a = @linear.Vec::of([0.0, 0.0, 0.0])
  let b = @linear.Vec::of([-1.0, 0.5, 0.5])
  assert_eq(close(car.distance(a, b), car.path(start=a, goal=b).length), true)
  let middle = car.interpolate(a, b, 0.5)
  assert_eq(
    close(
      middle.get(0),
      car.path(start=a, goal=b).at(car.distance(a, b) * 0.5).get(0),
    ),
    true,
  )
  assert_eq(close(car.interpolate(a, b, 0.0).get(0), 0.0), true)
  assert_eq(close(car.interpolate(a, b, 1.0).get(1), 0.5), true)
}

///|
test "the reeds shepp space samples inside its box" {
  let car = @planning.ReedsShepp::new(turning_radius=1.0, extent=2.0)
  let rng = @rand.seeded(b"reeds001")
  for _ in 0..<50 {
    let pose = car.sample(rng)
    assert_eq(pose.get(0).abs() <= 2.0, true)
    assert_eq(pose.get(1).abs() <= 2.0, true)
    assert_eq(pose.get(2).abs() <= 3.141592653589793, true)
  }
}

///|
test "a sampling planner plans inside the reeds shepp space" {
  let space : &@contract.Space = @planning.ReedsShepp::new(
    turning_radius=0.6,
    extent=2.0,
  )
  let validity : &@contract.Validity = Corridor::{ extent: 2.0, }
  let planner : &@contract.Planner = @planning.RrtStar::new(
    space~,
    validity~,
    step=0.4,
    goal_bias=0.1,
    neighbourhood=0.8,
  )
  let path = planner.plan(
    @linear.Vec::of([-1.5, 0.0, 0.0]),
    @linear.Vec::of([1.5, 0.0, 0.0]),
    { iterations: 400, seconds: None, },
  )
  assert_eq(path_is_free(validity, space, path), true)
}
