// A grid is a graph, and the graph package already searches one: this is the
// bridge, not a second implementation. It also exposes the grid as a planning
// space so a sampling planner can search a lattice.

///|
/// Which search a grid query runs.
#derive(Debug)
pub(all) enum GridSearch {
  ///
  /// Uniform-cost search from the start.
  Dijkstra
  ///
  /// The same with a Euclidean estimate.
  AStar
  ///
  /// Jump point search: A* that skips symmetric runs.
  Jps
  ///
  /// Any-angle search that keeps the shortest line of sight.
  ThetaStar
}

///|
/// The shortest path between two grid cells, as node indices, from the grid
/// package's own search.
pub fn grid_path(
  grid : @graph_grid.Grid,
  start~ : Int32,
  goal~ : Int32,
  search~ : GridSearch = AStar,
) -> Array[Int32] raise @linear.Refused {
  let found = match search {
    Dijkstra => dijkstra_to(grid, start, goal)
    AStar => grid.a_star(start, goal)
    Jps => grid.jps(start, goal)
    ThetaStar => grid.theta_star(start, goal)
  }
  match found {
    Some(cells) => {
      let out = Array::make(cells.length(), 0)
      for i in 0..<cells.length() {
        out[i] = cells[i]
      }
      out
    }
    None => raise @linear.Refused::Unreachable
  }
}

///|
/// The path a uniform-cost search finds, read back from the distances the grid
/// package computes: from the goal, step to whichever neighbour is exactly one
/// edge cheaper.
fn dijkstra_to(
  grid : @graph_grid.Grid,
  start : Int32,
  goal : Int32,
) -> Array[Int32]? {
  let distance = grid.dijkstra(start)
  if goal >= distance.length() || !distance[goal].is_finite() {
    return None
  }
  let reversed : Array[Int32] = [goal]
  let mut cursor = goal
  while cursor != start {
    let neighbours = grid.neighbors(cursor)
    let mut stepped = false
    for i in 0..<neighbours.length() {
      let candidate = neighbours[i]
      let edge = grid.step_cost(cursor, candidate)
      if (distance[candidate] + edge - distance[cursor]).abs() < 1.0e-9 {
        reversed.push(candidate)
        cursor = candidate
        stepped = true
        break
      }
    }
    if !stepped {
      return None
    }
  }
  let out = Array::make(reversed.length(), 0)
  for i in 0..<reversed.length() {
    out[i] = reversed[reversed.length() - 1 - i]
  }
  Some(out)
}

///|
/// A grid used as a planning space, so a sampling planner can search a lattice.
pub(all) struct GridSpace {
  grid : @graph_grid.Grid
  resolution : Double
}

///|
/// A space whose states are grid cells scaled by `resolution`.
pub fn GridSpace::new(
  grid : @graph_grid.Grid,
  resolution~ : Double = 1.0,
) -> GridSpace raise @linear.Refused {
  if resolution <= 0.0 {
    raise @linear.Refused::Gain(got=resolution)
  }
  { grid, resolution, }
}

///|
/// Whether the cell holding `x` is free.
fn GridSpace::free(self : GridSpace, x : @linear.Vec) -> Bool {
  let column = (x.get(0) / self.resolution).round().to_int()
  let row = (x.get(1) / self.resolution).round().to_int()
  if column < 0 ||
    row < 0 ||
    column >= self.grid.width() ||
    row >= self.grid.height() {
    return false
  }
  !self.grid.is_blocked(column, row)
}

///|
pub impl @contract.Space for GridSpace with fn sample(self, rng) {
  let column = (@rand.random(rng) * self.grid.width().to_double()).to_int()
  let row = (@rand.random(rng) * self.grid.height().to_double()).to_int()
  @linear.Vec::of([
    column.to_double() * self.resolution,
    row.to_double() * self.resolution,
  ])
}

///|
pub impl @contract.Space for GridSpace with fn distance(self, a, b) {
  ignore(self)
  a.sub(b).norm()
}

///|
pub impl @contract.Space for GridSpace with fn interpolate(self, a, b, t) {
  ignore(self)
  a.add(b.sub(a).scale(t))
}

///|
pub impl @contract.Validity for GridSpace with fn valid(self, x) {
  self.free(x)
}

///|
pub extend GridSpace with @contract.Space::{sample, distance, interpolate}

///|
pub extend GridSpace with @contract.Validity::{valid, motion}

///|
pub extend GridSearch with @debug.Debug::{to_repr}
