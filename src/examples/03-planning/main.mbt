// Shortest-path search on a 2-D grid: build a wall with a gap, then run both
// Dijkstra and A* from the bottom-left to the bottom-right cell and print the
// paths.
//
//     moon run src/examples/03-planning

///|
#coverage.skip
fn main {
  // A 10x10 grid with a vertical wall down the middle, gap in the top row.
  let blocked : Array[Int32] = []
  for row in 1..<10 {
    blocked.push(row * 10 + 5)
  }
  let grid = @planning.Grid::grid(10, 10, blocked_cells=Some(blocked))
  let start = grid.index(9, 0)
  let goal = grid.index(9, 9)
  let d = @planning.dijkstra(grid, start, goal).unwrap()
  let a = @planning.astar(grid, start, goal).unwrap()
  println(
    "dijkstra : " +
    d.length().to_string() +
    " cells, " +
    (d.length() - 1).to_string() +
    " steps",
  )
  println(
    "astar    : " +
    a.length().to_string() +
    " cells, " +
    (a.length() - 1).to_string() +
    " steps",
  )
  println("dijkstra path (row, col):")
  for cell in d.iter() {
    println(
      "  (" + (cell / 10).to_string() + ", " + (cell % 10).to_string() + ")",
    )
  }
}
