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
/// Rewire the neighbours of a freshly added node through it when that is cheaper.
///
/// Both star planners grow the same tree and use the same rule, so the rule lives
/// here: a neighbour is adopted only if reaching it through `candidate` costs
/// less than its current cost and the motion back to it is free. Reparenting also
/// carries the new cost down that neighbour's subtree.
fn rewire(
  space : &@contract.Space,
  validity : &@contract.Validity,
  tree : Tree,
  candidate : @linear.Vec,
  index : Int32,
  parent : Int32,
  cost : Double,
  neighbourhood : Double,
) -> Unit {
  for i in 0..<tree.nodes.length() {
    if i == index || i == parent {
      continue
    }
    let edge = space.distance(tree.nodes[i], candidate)
    if edge > neighbourhood {
      continue
    }
    let through = cost + edge
    if through < tree.costs[i] &&
      validity.motion(space, candidate, tree.nodes[i]) {
      tree.reparent(i, index, through)
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

///|
test "reparenting carries the new cost down the whole subtree" {
  // A node's cost is its parent's plus its edge. When an ancestor is rewired to
  // a cheaper parent, every descendant inherits the difference: a tree that
  // updated only the rewired node would let the search choose later parents on
  // costs those nodes no longer have.
  let tree = Tree::new(@linear.Vec::of([0.0, 0.0]))
  let a = tree.push(@linear.Vec::of([1.0, 0.0]), 0, 1.0)
  let b = tree.push(@linear.Vec::of([2.0, 0.0]), a, 3.0)
  let c = tree.push(@linear.Vec::of([3.0, 0.0]), b, 6.0)
  let sibling = tree.push(@linear.Vec::of([1.0, 1.0]), 0, 7.0)
  let shortcut = tree.push(@linear.Vec::of([0.5, 0.0]), 0, 0.2)
  tree.reparent(a, shortcut, 0.4)
  assert_eq(tree.parents[a], shortcut)
  assert_eq((tree.costs[a] - 0.4).abs() < 1.0e-9, true)
  assert_eq((tree.costs[b] - 2.4).abs() < 1.0e-9, true)
  assert_eq((tree.costs[c] - 5.4).abs() < 1.0e-9, true)
  // A node outside the subtree keeps the cost it earned.
  assert_eq((tree.costs[sibling] - 7.0).abs() < 1.0e-9, true)
  // The path to the leaf still runs through the rewired ancestor.
  let route = tree.path(c)
  assert_eq(route.length(), 5)
  assert_eq((route[4].get(0) - 3.0).abs() < 1.0e-9, true)
}

// --- white-box checks ------------------------------------------------------
//
// These live beside the tree rather than in a `_test.mbt` file: the tree is
// private to this package, and the search's rewiring rule is a property of that
// tree, not of the public planner result.

///|
/// A plane with nothing in it, for the checks below.
priv struct Flat {}

///|
impl @contract.Space for Flat with fn sample(self, rng) {
  ignore(self)
  ignore(rng)
  @linear.Vec::of([0.0, 0.0])
}

///|
impl @contract.Space for Flat with fn distance(self, a, b) {
  ignore(self)
  a.sub(b).norm()
}

///|
impl @contract.Space for Flat with fn interpolate(self, a, b, t) {
  ignore(self)
  a.add(b.sub(a).scale(t))
}

///|
impl @contract.Validity for Flat with fn valid(self, x) {
  ignore(self)
  ignore(x)
  true
}

///|
test "rewiring adopts a cheaper neighbour and carries its subtree" {
  let space : &@contract.Space = Flat::{ }
  let validity : &@contract.Validity = Flat::{ }
  let tree = Tree::new(@linear.Vec::of([0.0, 0.0]))
  // A is reached by an expensive detour, and B hangs off A outside the radius.
  let a = tree.push(@linear.Vec::of([0.0, 2.0]), 0, 8.0)
  let b = tree.push(@linear.Vec::of([0.0, 5.0]), a, 11.0)
  let candidate = @linear.Vec::of([0.0, 1.9])
  let index = tree.push(candidate, 0, 1.9)
  rewire(space, validity, tree, candidate, index, 0, 1.9, 1.0)
  assert_eq(tree.parents[a], index)
  assert_eq((tree.costs[a] - 2.0).abs() < 1.0e-9, true)
  // B is too far to be adopted itself, but it is a descendant of A, so it
  // inherits the six metres A just shed.
  assert_eq(tree.parents[b], a)
  assert_eq((tree.costs[b] - 5.0).abs() < 1.0e-9, true)
}
