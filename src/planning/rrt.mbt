// The rapidly-exploring random tree and its refinements. Each one grows the same
// tree and differs in how it picks the parent and when it stops.

///|
/// A rapidly-exploring random tree.
///
/// `step` is the length of the edge a new sample is allowed to reach, and
/// `goal_bias` the chance that a sample is the goal itself rather than a random
/// state. The generator is seeded from the planner, so a run replays exactly.
pub(all) struct Rrt {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
  rng : &@rand_contract.Rng
}

///|
/// An RRT that extends by `step` and steers to the goal once every
/// `1 / goal_bias` samples.
pub fn Rrt::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
  seed~ : Int32 = 0,
) -> Rrt raise @linear.Refused {
  if step <= 0.0 {
    raise @linear.Refused::Gain(got=step)
  }
  if goal_bias < 0.0 || goal_bias > 1.0 {
    raise @linear.Refused::Range(got=goal_bias, low=0.0, high=1.0)
  }
  { space, validity, step, goal_bias, rng: @rand.seeded(seed_bytes(seed)), }
}

///|
pub impl @contract.Planner for Rrt with fn plan(self, start, goal, budget) {
  let tree = Tree::new(start)
  for _ in 0..<budget.iterations {
    let target = if unit(self.rng) < self.goal_bias {
      goal
    } else {
      self.space.sample(self.rng)
    }
    let nearest = tree.nearest(self.space, target)
    let candidate = steer(self.space, tree.nodes[nearest], target, self.step)
    if !self.validity.motion(self.space, tree.nodes[nearest], candidate) {
      continue
    }
    let cost = tree.costs[nearest] +
      self.space.distance(tree.nodes[nearest], candidate)
    let index = tree.push(candidate, nearest, cost)
    if self.space.distance(candidate, goal) <= self.step &&
      self.validity.motion(self.space, candidate, goal) {
      let finish = tree.push(
        goal,
        index,
        cost + self.space.distance(candidate, goal),
      )
      return tree.path(finish)
    }
  }
  raise @linear.Refused::Exceeded(
    limit=budget.iterations,
    got=budget.iterations,
  )
}

///|
pub extend Rrt with @contract.Planner::{plan}

///|
/// An RRT that rewires the tree as it grows, so the path converges on the
/// shortest one inside the budget.
pub(all) struct RrtStar {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
  neighbourhood : Double
  rng : &@rand_contract.Rng
}

///|
/// An RRT* with a rewiring radius.
pub fn RrtStar::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
  neighbourhood~ : Double = 0.3,
  seed~ : Int32 = 0,
) -> RrtStar raise @linear.Refused {
  if step <= 0.0 {
    raise @linear.Refused::Gain(got=step)
  }
  if neighbourhood <= 0.0 {
    raise @linear.Refused::Gain(got=neighbourhood)
  }
  {
    space,
    validity,
    step,
    goal_bias,
    neighbourhood,
    rng: @rand.seeded(seed_bytes(seed)),
  }
}

///|
pub impl @contract.Planner for RrtStar with fn plan(self, start, goal, budget) {
  let tree = Tree::new(start)
  let mut best : Array[@linear.Vec]? = None
  let mut best_cost = 1.0e30
  for _ in 0..<budget.iterations {
    let target = if unit(self.rng) < self.goal_bias {
      goal
    } else {
      self.space.sample(self.rng)
    }
    let nearest = tree.nearest(self.space, target)
    let candidate = steer(self.space, tree.nodes[nearest], target, self.step)
    if !self.validity.motion(self.space, tree.nodes[nearest], candidate) {
      continue
    }
    // The best parent inside the radius, not merely the nearest one.
    let mut parent = nearest
    let mut cost = tree.costs[nearest] +
      self.space.distance(tree.nodes[nearest], candidate)
    for i in 0..<tree.nodes.length() {
      if self.space.distance(tree.nodes[i], candidate) > self.neighbourhood {
        continue
      }
      let through = tree.costs[i] +
        self.space.distance(tree.nodes[i], candidate)
      if through < cost &&
        self.validity.motion(self.space, tree.nodes[i], candidate) {
        parent = i
        cost = through
      }
    }
    let index = tree.push(candidate, parent, cost)
    // Rewiring: a neighbour that is cheaper through the new node changes parent.
    for i in 0..<tree.nodes.length() {
      if i == index || i == parent {
        continue
      }
      let edge = self.space.distance(tree.nodes[i], candidate)
      if edge > self.neighbourhood {
        continue
      }
      let through = cost + edge
      if through < tree.costs[i] &&
        self.validity.motion(self.space, candidate, tree.nodes[i]) {
        tree.parents[i] = index
        tree.costs[i] = through
      }
    }
    let to_goal = self.space.distance(candidate, goal)
    if cost + to_goal < best_cost &&
      self.validity.motion(self.space, candidate, goal) {
      let finish = tree.push(goal, index, cost + to_goal)
      best = Some(tree.path(finish))
      best_cost = cost + to_goal
    }
  }
  match best {
    Some(path) => path
    None =>
      raise @linear.Refused::Exceeded(
        limit=budget.iterations,
        got=budget.iterations,
      )
  }
}

///|
pub extend RrtStar with @contract.Planner::{plan}

///|
/// An RRT grown from both ends at once: the two trees take turns reaching for
/// each other, which closes a narrow passage far faster than one tree alone.
pub(all) struct RrtConnect {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  rng : &@rand_contract.Rng
}

///|
/// A bidirectional RRT.
pub fn RrtConnect::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  seed~ : Int32 = 0,
) -> RrtConnect raise @linear.Refused {
  if step <= 0.0 {
    raise @linear.Refused::Gain(got=step)
  }
  { space, validity, step, rng: @rand.seeded(seed_bytes(seed)), }
}

///|
/// Extend `tree` towards `target` one step; `Some` with the new index when the
/// step was allowed.
fn extend_once(
  tree : Tree,
  space : &@contract.Space,
  validity : &@contract.Validity,
  target : @linear.Vec,
  step : Double,
) -> Int32? {
  let nearest = tree.nearest(space, target)
  let candidate = steer(space, tree.nodes[nearest], target, step)
  if !validity.motion(space, tree.nodes[nearest], candidate) {
    return None
  }
  let cost = tree.costs[nearest] +
    space.distance(tree.nodes[nearest], candidate)
  Some(tree.push(candidate, nearest, cost))
}

///|
/// Extend `tree` towards `target` as far as it can go, one step at a time.
fn connect_towards(
  tree : Tree,
  space : &@contract.Space,
  validity : &@contract.Validity,
  target : @linear.Vec,
  step : Double,
) -> Int32? {
  let mut last : Int32? = None
  let mut reached = false
  while !reached {
    let nearest = tree.nearest(space, target)
    let distance = space.distance(tree.nodes[nearest], target)
    let candidate = if distance <= step {
      target.clone()
    } else {
      space.interpolate(tree.nodes[nearest], target, step / distance)
    }
    if !validity.motion(space, tree.nodes[nearest], candidate) {
      break
    }
    let cost = tree.costs[nearest] +
      space.distance(tree.nodes[nearest], candidate)
    last = Some(tree.push(candidate, nearest, cost))
    reached = distance <= step
  }
  last
}

///|
pub impl @contract.Planner for RrtConnect with fn plan(
  self,
  start,
  goal,
  budget,
) {
  let from_start = Tree::new(start)
  let from_goal = Tree::new(goal)
  for iteration in 0..<budget.iterations {
    let target = self.space.sample(self.rng)
    let grown = extend_once(
      from_start,
      self.space,
      self.validity,
      target,
      self.step,
    )
    match grown {
      None => ()
      Some(index) =>
        match
          connect_towards(
            from_goal,
            self.space,
            self.validity,
            from_start.nodes[index],
            self.step,
          ) {
          None => ()
          Some(other) =>
            if self.space.distance(
                from_start.nodes[index],
                from_goal.nodes[other],
              ) <
              1.0e-9 {
              let head = from_start.path(index)
              let tail = from_goal.path(other)
              let joined = Array::make(head.length() + tail.length() - 1, start)
              for i in 0..<head.length() {
                joined[i] = head[i]
              }
              for i in 1..<tail.length() {
                joined[head.length() - 1 + i] = tail[tail.length() - 1 - i]
              }
              return joined
            }
        }
    }
    ignore(iteration)
  }
  raise @linear.Refused::Exceeded(
    limit=budget.iterations,
    got=budget.iterations,
  )
}

///|
pub extend RrtConnect with @contract.Planner::{plan}

///|
/// An RRT* that only samples where a path shorter than the best one could still
/// run: the ellipse whose foci are the start and the goal.
pub(all) struct InformedRrtStar {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
  neighbourhood : Double
  rng : &@rand_contract.Rng
}

///|
/// An informed RRT* seeded from the best path found so far.
pub fn InformedRrtStar::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
  neighbourhood~ : Double = 0.3,
  seed~ : Int32 = 0,
) -> InformedRrtStar raise @linear.Refused {
  if step <= 0.0 {
    raise @linear.Refused::Gain(got=step)
  }
  if neighbourhood <= 0.0 {
    raise @linear.Refused::Gain(got=neighbourhood)
  }
  {
    space,
    validity,
    step,
    goal_bias,
    neighbourhood,
    rng: @rand.seeded(seed_bytes(seed)),
  }
}

///|
pub impl @contract.Planner for InformedRrtStar with fn plan(
  self,
  start,
  goal,
  budget,
) {
  let tree = Tree::new(start)
  let mut best : Array[@linear.Vec]? = None
  let mut best_cost = 1.0e30
  for _ in 0..<budget.iterations {
    // Rejection sampling against the ellipse: a state whose two distances add up
    // to more than the best path cannot lie on a shorter one.
    let mut target = self.space.sample(self.rng)
    let mut attempts = 0
    while best_cost < 1.0e29 && attempts < 64 {
      if self.space.distance(start, target) + self.space.distance(target, goal) <=
        best_cost {
        break
      }
      target = self.space.sample(self.rng)
      attempts = attempts + 1
    }
    let nearest = tree.nearest(self.space, target)
    let candidate = steer(self.space, tree.nodes[nearest], target, self.step)
    if !self.validity.motion(self.space, tree.nodes[nearest], candidate) {
      continue
    }
    let mut parent = nearest
    let mut cost = tree.costs[nearest] +
      self.space.distance(tree.nodes[nearest], candidate)
    for i in 0..<tree.nodes.length() {
      if self.space.distance(tree.nodes[i], candidate) > self.neighbourhood {
        continue
      }
      let through = tree.costs[i] +
        self.space.distance(tree.nodes[i], candidate)
      if through < cost &&
        self.validity.motion(self.space, tree.nodes[i], candidate) {
        parent = i
        cost = through
      }
    }
    let index = tree.push(candidate, parent, cost)
    for i in 0..<tree.nodes.length() {
      if i == index || i == parent {
        continue
      }
      let edge = self.space.distance(tree.nodes[i], candidate)
      if edge > self.neighbourhood {
        continue
      }
      let through = cost + edge
      if through < tree.costs[i] &&
        self.validity.motion(self.space, candidate, tree.nodes[i]) {
        tree.parents[i] = index
        tree.costs[i] = through
      }
    }
    let to_goal = self.space.distance(candidate, goal)
    if cost + to_goal < best_cost &&
      self.validity.motion(self.space, candidate, goal) {
      let finish = tree.push(goal, index, cost + to_goal)
      best = Some(tree.path(finish))
      best_cost = cost + to_goal
    }
  }
  match best {
    Some(path) => path
    None =>
      raise @linear.Refused::Exceeded(
        limit=budget.iterations,
        got=budget.iterations,
      )
  }
}

///|
pub extend InformedRrtStar with @contract.Planner::{plan}
