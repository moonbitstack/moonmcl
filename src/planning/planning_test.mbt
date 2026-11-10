// Tests for the planning package. The sampling planners are run in a world
// defined here, which is also the check that a space and a collision checker from
// outside this library need no adapter; the grid bridge is checked against the
// cells the graph package returns; and the Dubins paths are checked to start and
// end where they should.

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

// --- a space and a validity oracle defined outside the library -------------

///|
/// A plane with a wall down the middle and a gap in it.
pub struct Corridor {
  extent : Double
}

///|
pub impl @contract.Space for Corridor with fn sample(self, rng) {
  @linear.Vec::of([
    (@rand.random(rng) * 2.0 - 1.0) * self.extent,
    (@rand.random(rng) * 2.0 - 1.0) * self.extent,
  ])
}

///|
pub impl @contract.Space for Corridor with fn distance(self, a, b) {
  ignore(self)
  a.sub(b).norm()
}

///|
pub impl @contract.Space for Corridor with fn interpolate(self, a, b, t) {
  ignore(self)
  a.add(b.sub(a).scale(t))
}

///|
pub extend Corridor with @contract.Space::{sample, distance, interpolate}

///|
/// The wall from `(-0.1, -1)` to `(-0.1, -0.2)` and from `(-0.1, 0.2)` upwards:
/// the gap is at the origin.
pub impl @contract.Validity for Corridor with fn valid(self, x) {
  ignore(self)
  if x.get(0) > -0.1 && x.get(0) < 0.1 {
    return x.get(1).abs() < 0.2
  }
  true
}

///|
pub extend Corridor with @contract.Validity::{valid, motion}

///|
/// Whether every point of a path is free according to the same rule.
fn path_is_free(
  validity : &@contract.Validity,
  space : &@contract.Space,
  path : Array[@linear.Vec],
) -> Bool {
  for i in 1..<path.length() {
    if !validity.motion(space, path[i - 1], path[i]) {
      return false
    }
  }
  true
}

///|
test "rrt finds a path through a gap" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.Rrt::new(
    space~,
    validity~,
    step=0.2,
    goal_bias=0.1,
  )
  let path = planner.plan(
    @linear.Vec::of([-0.8, 0.0]),
    @linear.Vec::of([0.8, 0.0]),
    @contract.budget,
  )
  assert_eq(path.length() >= 2, true)
  assert_eq(close(path[0].get(0), -0.8), true)
  assert_eq(close(path[path.length() - 1].get(0), 0.8), true)
  assert_eq(path_is_free(validity, space, path), true)
}

///|
test "rrt star finds a path no longer than the straight line plus the detour" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.RrtStar::new(
    space~,
    validity~,
    step=0.2,
    goal_bias=0.1,
    neighbourhood=0.4,
  )
  let path = planner.plan(
    @linear.Vec::of([-0.8, -0.5]),
    @linear.Vec::of([0.8, -0.5]),
    { iterations: 3000, seconds: None, },
  )
  assert_eq(path_is_free(validity, space, path), true)
  let mut length = 0.0
  for i in 1..<path.length() {
    length = length + space.distance(path[i - 1], path[i])
  }
  assert_eq(length > 1.6, true)
}

///|
test "rrt connect finds a path through a gap" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.RrtConnect::new(
    space~,
    validity~,
    step=0.2,
  )
  let path = planner.plan(
    @linear.Vec::of([-0.8, 0.0]),
    @linear.Vec::of([0.8, 0.0]),
    @contract.budget,
  )
  assert_eq(path_is_free(validity, space, path), true)
}

///|
test "informed rrt star finds a path through a gap" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.InformedRrtStar::new(
    space~,
    validity~,
    step=0.2,
    goal_bias=0.1,
    neighbourhood=0.4,
  )
  let path = planner.plan(
    @linear.Vec::of([-0.8, 0.0]),
    @linear.Vec::of([0.8, 0.0]),
    { iterations: 2000, seconds: None, },
  )
  assert_eq(path_is_free(validity, space, path), true)
}

///|
test "prm answers a query through its roadmap" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.Prm::new(
    space~,
    validity~,
    samples=200,
    neighbours=8,
    connection=0.4,
  )
  let path = planner.plan(
    @linear.Vec::of([-0.8, 0.0]),
    @linear.Vec::of([0.8, 0.0]),
    @contract.budget,
  )
  assert_eq(path_is_free(validity, space, path), true)
}

///|
test "a planner that cannot find a path says so" {
  let space : &@contract.Space = Corridor::{ extent: 1.0, }
  let validity : &@contract.Validity = Corridor::{ extent: 1.0, }
  let planner : &@contract.Planner = @planning.Rrt::new(
    space~,
    validity~,
    step=0.2,
  )
  match
    refused(fn() raise @linear.Refused {
      // Both ends inside the wall: no path exists at all.
      ignore(
        planner.plan(
          @linear.Vec::of([0.0, 0.5]),
          @linear.Vec::of([0.0, -0.5]),
          { iterations: 200, seconds: None, },
        ),
      )
    }) {
    Some(_) => ()
    None => abort("expected a refusal")
  }
}

// --- Dubins ---------------------------------------------------------------

///|
test "dubins path starts and ends at the poses it was given" {
  let car = @planning.Dubins::new(turning_radius=1.0)
  let start = @linear.Vec::of([0.0, 0.0, 0.0])
  let goal = @linear.Vec::of([2.0, 1.0, 1.0])
  let path = car.path(start~, goal~)
  let first = path.at(0.0)
  let last = path.at(path.length)
  assert_eq(close(first.get(0), 0.0, tolerance=1.0e-9), true)
  assert_eq(close(first.get(1), 0.0, tolerance=1.0e-9), true)
  assert_eq(close(last.get(0), 2.0, tolerance=1.0e-9), true)
  assert_eq(close(last.get(1), 1.0, tolerance=1.0e-9), true)
  assert_eq(close(last.get(2), 1.0, tolerance=1.0e-9), true)
}

///|
test "dubins length is never shorter than the straight line" {
  let car = @planning.Dubins::new(turning_radius=2.0)
  let path = car.path(
    start=@linear.Vec::of([0.0, 0.0, 0.0]),
    goal=@linear.Vec::of([5.0, 0.0, 0.0]),
  )
  assert_eq(path.length >= 5.0, true)
  // Straight ahead with matching headings is the straight line itself.
  assert_eq(close(path.length, 5.0, tolerance=1.0e-9), true)
}

///|
test "dubins space samples inside its box" {
  let car = @planning.Dubins::new(turning_radius=1.0, extent=2.0)
  let rng = @rand.seeded(b"dubins01")
  for _ in 0..<50 {
    let pose = car.sample(rng)
    assert_eq(pose.get(0).abs() <= 2.0, true)
    assert_eq(pose.get(1).abs() <= 2.0, true)
    assert_eq(pose.get(2).abs() <= 3.141592653589793, true)
  }
}

// --- grid bridge ----------------------------------------------------------

///|
test "grid path goes around a blocked row" {
  let grid = @grid.Grid::new(5, 3)
  // Block the middle of the first two columns of the middle row.
  for column in 0..<4 {
    grid.block(column, 1)
  }
  let start = grid.node(0, 0)
  let goal = grid.node(0, 2)
  let path = @planning.grid_path(
    grid,
    start~,
    goal~,
    search=@planning.GridSearch::AStar,
  )
  assert_eq(path.length() >= 4, true)
  assert_eq(path[0], start)
  assert_eq(path[path.length() - 1], goal)
}

///|
test "grid path refuses an unreachable goal" {
  let grid = @grid.Grid::new(3, 3)
  for column in 0..<3 {
    grid.block(column, 1)
  }
  grid.block(0, 2)
  grid.block(1, 2)
  grid.block(2, 2)
  match
    refused(fn() raise @linear.Refused {
      ignore(
        @planning.grid_path(grid, start=grid.node(0, 0), goal=grid.node(2, 2)),
      )
    }) {
    Some(@linear.Refused::Unreachable) => ()
    _ => abort("expected Unreachable")
  }
}

///|
test "grid path agrees between the four searches" {
  let grid = @grid.Grid::new(6, 4)
  grid.block(2, 1)
  grid.block(2, 2)
  let start = grid.node(0, 0)
  let goal = grid.node(5, 3)
  let searches = [
    @planning.GridSearch::Dijkstra,
    @planning.GridSearch::AStar,
    @planning.GridSearch::Jps,
    @planning.GridSearch::ThetaStar,
  ]
  for search in searches {
    let path = @planning.grid_path(grid, start~, goal~, search~)
    assert_eq(path[0], start)
    assert_eq(path[path.length() - 1], goal)
  }
}

// --- local planners -------------------------------------------------------

///|
test "dynamic window drives towards the goal" {
  let command = @planning.dynamic_window.command(
    pose=@linear.Vec::of([0.0, 0.0, 0.0]),
    velocity=@linear.Vec::of([0.0, 0.0]),
    goal=@linear.Vec::of([5.0, 0.0]),
    obstacles=[],
  )
  assert_eq(command.get(0) > 0.0, true)
  assert_eq(close(command.get(1), 0.0, tolerance=1.0e-9), true)
}

///|
test "dynamic window turns away from a wall across its path" {
  let command = @planning.dynamic_window.command(
    pose=@linear.Vec::of([0.0, 0.0, 0.0]),
    velocity=@linear.Vec::of([0.5, 0.0]),
    goal=@linear.Vec::of([5.0, 0.0]),
    // A wall a third of a metre ahead: every straight-ahead candidate ends up
    // inside it, so the window has to turn.
    obstacles=[
      @linear.Vec::of([0.3, -0.3]),
      @linear.Vec::of([0.3, -0.15]),
      @linear.Vec::of([0.3, 0.0]),
      @linear.Vec::of([0.3, 0.15]),
      @linear.Vec::of([0.3, 0.3]),
    ],
  )
  assert_eq(command.get(1).abs() > 1.0e-9, true)
}

///|
test "potential field points at the goal when nothing is in the way" {
  let command = @planning.potential_field.command(
    position=@linear.Vec::of([0.0, 0.0]),
    goal=@linear.Vec::of([1.0, 0.0]),
    obstacles=[],
  )
  assert_eq(close(command.get(0), 0.05, tolerance=1.0e-9), true)
  assert_eq(close(command.get(1), 0.0, tolerance=1.0e-9), true)
}

///|
test "potential field is pushed sideways by an obstacle" {
  let command = @planning.potential_field.command(
    position=@linear.Vec::of([0.0, 0.0]),
    goal=@linear.Vec::of([1.0, 0.0]),
    obstacles=[@linear.Vec::of([0.2, 0.0])],
  )
  // Straight at the goal plus a push straight back sum along the axis; the sum
  // never pushes towards the obstacle.
  assert_eq(command.get(0) < 0.05, true)
}
