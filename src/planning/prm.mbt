// The probabilistic roadmap: a graph sampled once, then queried many times. It is
// the planner to build when the same world is asked for many paths, because the
// expensive part is the sampling and the validity checks, not the search.

///|
/// A probabilistic roadmap.
pub(all) struct Prm {
  space : &@contract.Space
  validity : &@contract.Validity
  samples : Int32
  neighbours : Int32
  connection : Double
  rng : &@rand_contract.Rng
}

///|
/// A roadmap with the given sample count and connection radius.
pub fn Prm::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  samples~ : Int32 = 500,
  neighbours~ : Int32 = 10,
  connection~ : Double = 1.0,
  seed~ : Int32 = 0,
) -> Prm raise @linear.Refused {
  if samples < 2 {
    raise @linear.Refused::Range(got=samples.to_double(), low=2.0, high=1.0e9)
  }
  if neighbours < 1 {
    raise @linear.Refused::Range(
      got=neighbours.to_double(),
      low=1.0,
      high=1.0e9,
    )
  }
  {
    space,
    validity,
    samples,
    neighbours,
    connection,
    rng: @rand.seeded(seed_bytes(seed)),
  }
}

///|
/// A roadmap: the nodes, and for each node the neighbours it can reach with the
/// cost of the edge.
priv struct Roadmap {
  nodes : Array[@linear.Vec]
  edges : Array[Array[(Int32, Double)]]
}

///|
/// Build the roadmap: free samples, each joined to its nearest valid neighbours.
fn Prm::roadmap(self : Prm) -> Roadmap {
  let nodes : Array[@linear.Vec] = []
  while nodes.length() < self.samples {
    let candidate = self.space.sample(self.rng)
    if self.validity.valid(candidate) {
      nodes.push(candidate)
    }
  }
  let edges = Array::make(nodes.length(), Array::make(0, (0, 0.0)))
  for i in 0..<nodes.length() {
    edges[i] = []
  }
  for i in 0..<nodes.length() {
    // The nearest few neighbours inside the connection radius, taken by sorting a
    // small candidate list rather than the whole node set.
    let distance = Array::make(nodes.length(), 0.0)
    let order = Array::make(nodes.length(), 0)
    for j in 0..<nodes.length() {
      distance[j] = if j == i {
        1.0e30
      } else {
        self.space.distance(nodes[i], nodes[j])
      }
      order[j] = j
    }
    for a in 0..<nodes.length() {
      let mut best = a
      for b in (a + 1)..<nodes.length() {
        if distance[order[b]] < distance[order[best]] {
          best = b
        }
      }
      let swap = order[a]
      order[a] = order[best]
      order[best] = swap
    }
    let mut accepted = 0
    for k in 0..<nodes.length() {
      if accepted >= self.neighbours || distance[order[k]] > self.connection {
        break
      }
      let target = order[k]
      if self.validity.motion(self.space, nodes[i], nodes[target]) {
        edges[i].push((target, distance[target]))
        accepted = accepted + 1
      }
    }
  }
  { nodes, edges, }
}

///|
/// The shortest path from `source` to `sink` in the roadmap, as node indices.
fn Roadmap::shortest(
  self : Roadmap,
  source : Int32,
  sink : Int32,
) -> Array[Int32]? raise @linear.Refused {
  let count = self.nodes.length()
  let distance = Array::make(count, 1.0e30)
  let previous = Array::make(count, -1)
  let settled = Array::make(count, false)
  distance[source] = 0.0
  let queue = @heap.DaryHeap::new_by(cmp=fn(
    a : (Double, Int32),
    b : (Double, Int32),
  ) {
    if a.0 < b.0 {
      -1
    } else if a.0 > b.0 {
      1
    } else {
      0
    }
  }) catch {
    _ => raise @linear.Refused::Unreachable
  }
  queue.push((0.0, source))
  while queue.length() > 0 {
    let (cost, node) = queue.pop().unwrap()
    if settled[node] {
      continue
    }
    settled[node] = true
    if node == sink {
      break
    }
    for edge in self.edges[node] {
      let (target, weight) = edge
      let through = cost + weight
      if through < distance[target] {
        distance[target] = through
        previous[target] = node
        queue.push((through, target))
      }
    }
  }
  if distance[sink] >= 1.0e29 {
    return None
  }
  let reversed : Array[Int32] = []
  let mut cursor = sink
  while cursor >= 0 {
    reversed.push(cursor)
    cursor = previous[cursor]
  }
  let out = Array::make(reversed.length(), 0)
  for i in 0..<reversed.length() {
    out[i] = reversed[reversed.length() - 1 - i]
  }
  Some(out)
}

///|
pub impl @contract.Planner for Prm with fn plan(self, start, goal, budget) {
  ignore(budget)
  let roadmap = self.roadmap()
  // The query states join the roadmap as extra nodes with their own edges.
  let nodes = Array::make(roadmap.nodes.length() + 2, start)
  for i in 0..<roadmap.nodes.length() {
    nodes[i] = roadmap.nodes[i]
  }
  nodes[roadmap.nodes.length()] = start
  nodes[roadmap.nodes.length() + 1] = goal
  let source = roadmap.nodes.length()
  let sink = roadmap.nodes.length() + 1
  let edges = Array::make(nodes.length(), Array::make(0, (0, 0.0)))
  for i in 0..<roadmap.edges.length() {
    edges[i] = roadmap.edges[i].copy()
  }
  edges[source] = []
  edges[sink] = []
  // Connect the two query states to their nearest valid nodes.
  for query in [source, sink] {
    let distance = Array::make(roadmap.nodes.length(), 0.0)
    let order = Array::make(roadmap.nodes.length(), 0)
    for j in 0..<roadmap.nodes.length() {
      distance[j] = self.space.distance(nodes[query], nodes[j])
      order[j] = j
    }
    for a in 0..<order.length() {
      let mut best = a
      for b in (a + 1)..<order.length() {
        if distance[order[b]] < distance[order[best]] {
          best = b
        }
      }
      let swap = order[a]
      order[a] = order[best]
      order[best] = swap
    }
    let mut accepted = 0
    for k in 0..<order.length() {
      if accepted >= self.neighbours || distance[order[k]] > self.connection {
        break
      }
      let target = order[k]
      if self.validity.motion(self.space, nodes[query], nodes[target]) {
        edges[query].push((target, distance[target]))
        edges[target].push((query, distance[target]))
        accepted = accepted + 1
      }
    }
  }
  let graph = { nodes, edges, }
  match graph.shortest(source, sink) {
    Some(indices) => {
      let out = Array::make(indices.length(), start)
      for i in 0..<indices.length() {
        out[i] = nodes[indices[i]]
      }
      out
    }
    None => raise @linear.Refused::Unreachable
  }
}

///|
pub extend Prm with @contract.Planner::{plan}
