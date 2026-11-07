// A* search on a unit-cost grid with a Manhattan-distance heuristic, which is
// admissible and consistent for 4-connected movement, so the first time the
// goal is popped it holds the optimal (fewest-step) path. Returns the path as
// an array of flat cell indices, or `None` when the goal is unreachable.

///|
/// A* search from `start` to `goal` with a Manhattan-distance heuristic, which
/// is admissible and consistent for 4-connected movement, so the first pop of
/// the goal holds a path of the fewest steps. Returns the path as flat cell
/// indices including both ends, or `None` when the goal is unreachable.
pub fn astar(g : Grid, start : Int32, goal : Int32) -> Array[Int32]? {
  if g.is_blocked(start) || g.is_blocked(goal) {
    return None
  }
  let g_score : @hashmap.HashMap[Int32, Int32] = HashMap([])
  let came_from : @hashmap.HashMap[Int32, Int32] = HashMap([])
  g_score.set(start, 0)
  let open = MinHeap::min_heap()
  open.push(manhattan(g, start, goal), start)
  let mut finished = false
  while open.pop() is Some((_, u)) {
    let gu = g_score.get(u).unwrap_or(0)
    if u == goal {
      finished = true
      break
    }
    for v in g.neighbors(u).iter() {
      let ng = gu + 1
      let prev = g_score.get(v)
      if prev is None || ng < prev.unwrap() {
        g_score.set(v, ng)
        came_from.set(v, u)
        open.push(ng + manhattan(g, v, goal), v)
      }
    }
  }
  if finished || g_score.get(goal) is Some(_) {
    Some(reconstruct_path(came_from, start, goal))
  } else {
    None
  }
}

///|
/// The Manhattan distance between the cells `a` and `b` on grid `g`.
fn manhattan(g : Grid, a : Int32, b : Int32) -> Int32 {
  let ar = a / g.cols
  let ac = a % g.cols
  let br = b / g.cols
  let bc = b % g.cols
  (ar - br).abs() + (ac - bc).abs()
}
