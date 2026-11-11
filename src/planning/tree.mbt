// The tree the sampling planners grow. A flat node array with a parent index and
// a cost keeps the nearest search and the rewiring cheap, and it is the same
// structure for the single-tree and the two-tree planners.

///|
/// A tree of states with their parents and the cost of reaching them.
priv struct Tree {
  nodes : Array[@linear.Vec]
  parents : Array[Int32]
  costs : Array[Double]
}

///|
/// A tree rooted at `root`.
fn Tree::new(root : @linear.Vec) -> Tree {
  { nodes: [root], parents: [-1], costs: [0.0], }
}

///|
/// The index of the node nearest to `target`.
fn Tree::nearest(
  self : Tree,
  space : &@contract.Space,
  target : @linear.Vec,
) -> Int32 {
  let mut best = 0
  let mut best_distance = space.distance(self.nodes[0], target)
  for i in 1..<self.nodes.length() {
    let distance = space.distance(self.nodes[i], target)
    if distance < best_distance {
      best_distance = distance
      best = i
    }
  }
  best
}

///|
/// Append a node with a parent and a cost, and answer its index.
fn Tree::push(
  self : Tree,
  node : @linear.Vec,
  parent : Int32,
  cost : Double,
) -> Int32 {
  self.nodes.push(node)
  self.parents.push(parent)
  self.costs.push(cost)
  self.nodes.length() - 1
}

///|
/// Give `index` a new parent and cost, and carry the difference down its subtree.
///
/// A node's cost is the cost of its parent plus its edge, so changing it changes
/// every descendant's; leaving them behind would let the search pick a parent on
/// a cost that node no longer has.
fn Tree::reparent(
  self : Tree,
  index : Int32,
  parent : Int32,
  cost : Double,
) -> Unit {
  let delta = cost - self.costs[index]
  self.parents[index] = parent
  self.costs[index] = cost
  if delta == 0.0 {
    return
  }
  let pending = [index]
  while pending.length() > 0 {
    let current = pending.pop().unwrap()
    for j in 0..<self.nodes.length() {
      if self.parents[j] == current {
        self.costs[j] = self.costs[j] + delta
        pending.push(j)
      }
    }
  }
}

///|
/// The path from the root to `index`, root first.
fn Tree::path(self : Tree, index : Int32) -> Array[@linear.Vec] {
  let reversed : Array[@linear.Vec] = []
  let mut cursor = index
  while cursor >= 0 {
    reversed.push(self.nodes[cursor])
    cursor = self.parents[cursor]
  }
  let out = Array::make(reversed.length(), self.nodes[0])
  for i in 0..<reversed.length() {
    out[i] = reversed[reversed.length() - 1 - i]
  }
  out
}

///|
/// The node that `step` from `from` towards `towards` reaches, and whether it
/// ends exactly on the target.
fn steer(
  space : &@contract.Space,
  from : @linear.Vec,
  towards : @linear.Vec,
  step : Double,
) -> @linear.Vec {
  let distance = space.distance(from, towards)
  if distance <= step || distance == 0.0 {
    towards.clone()
  } else {
    space.interpolate(from, towards, step / distance)
  }
}

///|
/// A deterministic seed from an integer, so a planner run is reproducible and a
/// test can replay it.
fn seed_bytes(seed : Int32) -> Bytes {
  let bytes = Array::make(8, b'\x00')
  let mut value = seed
  for i in 0..<8 {
    bytes[i] = (value % 256).to_byte()
    value = value / 256
  }
  Bytes::from_array(bytes)
}

///|
/// A uniform draw from the generator in `[0, 1)`.
fn unit(rng : &@rand_contract.Rng) -> Double {
  @rand.random(rng)
}
