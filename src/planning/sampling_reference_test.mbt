// A sampling planner against a mature robotics toolbox on the same map. The map
// and the cost of the reference route come from
// `scripts/vectors/gen_pythonrobotics.py`.
//
// The two planners are not expected to return the same route: both draw random
// samples, from different generators, and a sampling planner's answer depends on
// the sequence it drew. What is compared is the order of magnitude of the cost and
// whether the route is one the map allows — which is what a planner is for.

///|
/// A square arena with circular obstacles.
struct Arena {
  extent : Double
  obstacles : Array[(Double, Double, Double)]
  clearance : Double
}

///|
impl @contract.Space for Arena with fn sample(self, rng) {
  @linear.Vec::of([
    @rand.random(rng) * self.extent,
    @rand.random(rng) * self.extent,
  ])
}

///|
impl @contract.Space for Arena with fn distance(self, a, b) {
  ignore(self)
  a.sub(b).norm()
}

///|
impl @contract.Space for Arena with fn interpolate(self, a, b, t) {
  ignore(self)
  a.add(b.sub(a).scale(t))
}

///|
impl @contract.Validity for Arena with fn valid(self, x) {
  for obstacle in self.obstacles {
    let (ox, oy, radius) = obstacle
    let dx = x.get(0) - ox
    let dy = x.get(1) - oy
    if (dx * dx + dy * dy).sqrt() < radius + self.clearance {
      return false
    }
  }
  true
}

///|
pub extend Arena with @contract.Space::{sample, distance, interpolate}

///|
pub extend Arena with @contract.Validity::{valid, motion}

///|
fn arena_of(case : SamplingCase) -> Arena {
  Arena::{
    extent: 10.0,
    obstacles: case.obstacles.map(fn(o) { (o[0], o[1], o[2]) }),
    clearance: 0.4,
  }
}

///|
fn route_length(space : &@contract.Space, path : Array[@linear.Vec]) -> Double {
  let mut total = 0.0
  for i in 1..<path.length() {
    total = total + space.distance(path[i - 1], path[i])
  }
  total
}

///|
test "a sampling planner finds a route of the reference's order of magnitude" {
  let cases = pythonrobotics_planning_cases()
  assert_eq(cases.length() >= 3, true)
  for case in cases {
    assert_eq(case.name.length() > 0, true)
    let arena = arena_of(case)
    let space : &@contract.Space = arena
    let validity : &@contract.Validity = arena
    let planner : &@contract.Planner = @planning.RrtStar::new(
      space~,
      validity~,
      step=1.0,
      goal_bias=0.1,
      neighbourhood=2.0,
      seed=7,
    )
    let path = planner.plan(
      @linear.Vec::of(case.start),
      @linear.Vec::of(case.goal),
      { iterations: 4000, seconds: None, },
    )
    // Feasibility: both ends, and every edge the map allows.
    assert_eq((path[0].get(0) - case.start[0]).abs() < 1.0e-12, true)
    let last = path[path.length() - 1]
    assert_eq((last.get(0) - case.goal[0]).abs() < 1.0e-12, true)
    assert_eq((last.get(1) - case.goal[1]).abs() < 1.0e-12, true)
    for i in 1..<path.length() {
      assert_eq(validity.motion(space, path[i - 1], path[i]), true)
    }
    // Cost: never shorter than the straight line, and within a factor of the
    // reference's route. The factor covers the different random streams; a route
    // that wandered or cut a corner would be caught by the two checks above.
    let cost = route_length(space, path)
    let direct = @math.hypot(
      case.goal[0] - case.start[0],
      case.goal[1] - case.start[1],
    )
    assert_eq(cost >= direct - 1.0e-9, true)
    assert_eq(cost <= case.cost * 2.5, true)
  }
}
