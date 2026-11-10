// A sampling planner searching a grid through the graph package's own search
// space, and the same grid searched directly.
//
//     moon run src/examples/04-planning

///|
#coverage.skip
fn main raise {
  let grid = @grid.Grid::new(12, 8)
  // A wall with a gap two thirds of the way along.
  for row in 0..<7 {
    grid.block(6, row)
  }
  let start = grid.node(0, 0)
  let goal = grid.node(11, 7)
  let path = @mctl.grid_path(
    grid,
    start~,
    goal~,
    search=@mctl.GridSearch::AStar,
  )
  println("cells " + path.length().to_string())
  for cell in path {
    println(
      "(" + grid.x(cell).to_string() + ", " + grid.y(cell).to_string() + ")",
    )
  }
}
