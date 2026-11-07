// Dijkstra's algorithm on a unit-cost grid: the shortest path is the
// fewest-step path. Returns the path as an array of flat cell indices from
// `start` to `goal`, or `None` when the goal is unreachable.

///|
/// Uniform-cost search from `start` to `goal`. On a unit-cost grid the shortest
/// path is the fewest-step one. Returns the path as flat cell indices including
/// both ends, or `None` when the goal is unreachable.
pub fn dijkstra(g : Grid, start : Int32, goal : Int32) -> Array[Int32]? {
  if g.is_blocked(start) || g.is_blocked(goal) {
    return None
  }
  let dist : @hashmap.HashMap[Int32, Int32] = HashMap([])
  let came_from : @hashmap.HashMap[Int32, Int32] = HashMap([])
  dist.set(start, 0)
  let open = MinHeap::min_heap()
  open.push(0, start)
  let mut finished = false
  while open.pop() is Some((_, u)) {
    let du = dist.get(u).unwrap_or(0)
    if u == goal {
      finished = true
      break
    }
    for v in g.neighbors(u).iter() {
      let nd = du + 1
      let prev = dist.get(v)
      if prev is None || nd < prev.unwrap() {
        dist.set(v, nd)
        came_from.set(v, u)
        open.push(nd, v)
      }
    }
  }
  if finished || dist.get(goal) is Some(_) {
    Some(reconstruct_path(came_from, start, goal))
  } else {
    None
  }
}

///|
fn reconstruct_path(
  came_from : @hashmap.HashMap[Int32, Int32],
  start : Int32,
  goal : Int32,
) -> Array[Int32] {
  let path : Array[Int32] = Array(capacity=1)
  path.push(goal)
  let mut cur = goal
  while cur != start {
    let parent = came_from.get(cur)
    guard parent is Some(_) else { break }
    path.push(parent.unwrap())
    cur = parent.unwrap()
  }
  path.rev_in_place()
  path
}
