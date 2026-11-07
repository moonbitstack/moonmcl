// Tests for the planning module: Dijkstra and A* on a 2-D grid. They check the
// fewest-step distance in an open grid (where it equals the Manhattan
// distance), obstacle avoidance, and the unreachable case.

///|
test "open grid path length equals manhattan distance" {
  let g = @planning.Grid::grid(10, 10)
  // Corner to corner on a 10x10 grid: 9 right + 9 down = 18 steps.
  let start = g.index(0, 0)
  let goal = g.index(9, 9)
  let d = @planning.dijkstra(g, start, goal)
  assert_eq(d is Some(_), true)
  assert_eq(d.unwrap().length() - 1, 18)
  let a = @planning.astar(g, start, goal)
  assert_eq(a is Some(_), true)
  assert_eq(a.unwrap().length() - 1, 18)
}

///|
test "path starts and ends at the right cells" {
  let g = @planning.Grid::grid(5, 5)
  let start = g.index(2, 2)
  let goal = g.index(4, 0)
  let d = @planning.dijkstra(g, start, goal).unwrap()
  assert_eq(d[0], start)
  assert_eq(d[d.length() - 1], goal)
}

///|
test "blocked goal is unreachable" {
  let g = @planning.Grid::grid(5, 5, blocked_cells=Some([24]))
  let start = g.index(0, 0)
  let goal = g.index(4, 4)
  assert_eq(@planning.dijkstra(g, start, goal), None)
  assert_eq(@planning.astar(g, start, goal), None)
}

///|
test "path goes around a wall" {
  // A vertical wall across the middle with a gap in the top row.
  let blocked : Array[Int32] = []
  for row in 1..<10 {
    blocked.push(row * 10 + 5)
  }
  let g = @planning.Grid::grid(10, 10, blocked_cells=Some(blocked))
  let start = g.index(9, 0)
  let goal = g.index(9, 9)
  let d = @planning.dijkstra(g, start, goal)
  assert_eq(d is Some(_), true)
  // The wall is 10 tall minus the gap row 0, so the path must reach row 0
  // (through the gap at (0, 5)).
  let path = d.unwrap()
  let mut reached_top = false
  for cell in path.iter() {
    if cell / 10 == 0 {
      reached_top = true
    }
  }
  assert_eq(reached_top, true)
}

///|
test "astar matches dijkstra distance" {
  // On a unit-cost grid A* with the (consistent) Manhattan heuristic returns
  // an optimal path, so its length must match Dijkstra's.
  let g = @planning.Grid::grid(8, 8)
  let start = g.index(7, 7)
  let goal = g.index(0, 3)
  let d = @planning.dijkstra(g, start, goal).unwrap()
  let a = @planning.astar(g, start, goal).unwrap()
  assert_eq(d.length(), a.length())
}
