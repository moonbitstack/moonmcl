// The three figures the showcase page added last: a set of axes running on one
// clock, a plan made inside a car's own state space, and a delta cell driven
// around a circle. Each returns one record per row and lets the page draw it.

///|
/// A wall with a gap in it, as a collision oracle for a planner the reader can
/// see working. The gap is at the origin, so a route has to thread it.
struct Wall {}

///|
impl @contract.Space for Wall with fn sample(self, rng) {
  ignore(self)
  @mctl.Vec::of([
    (@rand.random(rng) * 2.0 - 1.0) * 2.5,
    (@rand.random(rng) * 2.0 - 1.0) * 2.5,
    @rand.random(rng) * 6.283185307179586 - 3.141592653589793,
  ])
}

///|
impl @contract.Space for Wall with fn distance(self, a, b) {
  ignore(self)
  a.sub(b).norm()
}

///|
impl @contract.Space for Wall with fn interpolate(self, a, b, t) {
  ignore(self)
  a.add(b.sub(a).scale(t))
}

///|
impl @contract.Validity for Wall with fn valid(self, x) {
  ignore(self)
  if x.get(0).abs() < 0.15 {
    return x.get(1).abs() < 0.35
  }
  true
}

///|
impl @contract.Validity for Wall with fn motion(self, space, a, b) {
  ignore(self)
  // A path between two poses is at least as long as the straight line between
  // them, so an edge shorter than the nearer end's distance to the wall cannot
  // reach the wall at all. That settles most edges of a tree grown in this arena
  // before the space is asked for anything, which matters here: inside a car's
  // space every interpolation is a shortest-path solve.
  let reach = ((a.get(0) - b.get(0)) * (a.get(0) - b.get(0)) +
  (a.get(1) - b.get(1)) * (a.get(1) - b.get(1))).sqrt()
  let nearer = if a.get(0).abs() < b.get(0).abs() {
    a.get(0).abs()
  } else {
    b.get(0).abs()
  }
  if reach < nearer - 0.15 {
    return true
  }
  // A fixed number of samples is not enough here: two poses half a metre apart
  // can be joined by metres of arc when their headings disagree, and eight
  // points over three metres step straight past a wall a third of a metre thick.
  // The space's own distance is the arc length, so it sets the step.
  let length = space.distance(a, b)
  let mut steps = (length / 0.1).ceil().to_int()
  if steps < 8 {
    steps = 8
  } else if steps > 64 {
    steps = 64
  }
  for i in 0..<steps {
    let t = (i + 1).to_double() / steps.to_double()
    if !self.valid(space.interpolate(a, b, t)) {
      return false
    }
  }
  true
}

///|
pub extend Wall with @contract.Space::{sample, distance, interpolate}

///|
pub extend Wall with @contract.Validity::{valid, motion}

///|
/// Two axes with different distances under the same limits, synchronised so both
/// arrive together. The columns are the shared clock, the position and velocity of
/// each axis, and the duration each axis would have had alone.
fn sync_trace(
  short_distance : Double,
  long_distance : Double,
  v_max : Double,
  a_max : Double,
  j_max : Double,
  samples : Int32,
) -> String {
  let short = @mctl.scurve(distance=short_distance, v_max~, a_max~, j_max~) catch {
    _ => return "ERR impossible"
  }
  let long = @mctl.scurve(distance=long_distance, v_max~, a_max~, j_max~) catch {
    _ => return "ERR impossible"
  }
  let set = @mctl.Synced::new(axes=[short, long]) catch {
    _ => return "ERR refused"
  }
  let rows : Array[String] = []
  let count = if samples < 2 { 2 } else { samples }
  let total = set.duration()
  for i in 0..<count {
    let t = total * i.to_double() / (count - 1).to_double()
    let pair = set.sample_all(t)
    rows.push(
      record([
        t,
        pair[0].position,
        pair[1].position,
        pair[0].velocity,
        pair[1].velocity,
        short.duration(),
        long.duration(),
      ]),
    )
  }
  table(rows)
}

///|
/// A route from one pose to another inside a car's own state space. `mode` picks
/// the space and the search: `0` the shortest Reeds-Shepp path, `1` the shortest
/// Dubins path, `2` a tree grown inside the Reeds-Shepp space around the wall.
fn space_trace(
  start_x : Double,
  start_y : Double,
  start_heading : Double,
  goal_x : Double,
  goal_y : Double,
  goal_heading : Double,
  radius : Double,
  mode : Int32,
  samples : Int32,
) -> String {
  let start = @mctl.Vec::of([start_x, start_y, start_heading])
  let goal = @mctl.Vec::of([goal_x, goal_y, goal_heading])
  let count = if samples < 2 { 2 } else { samples }
  if mode == 1 {
    let car = @mctl.Dubins::new(turning_radius=radius) catch {
      _ => return "ERR refused"
    }
    let path = car.path(start~, goal~) catch { _ => return "ERR unreachable" }
    let rows : Array[String] = []
    for i in 0..<count {
      let pose = path.at(path.length * i.to_double() / (count - 1).to_double())
      rows.push(record([pose.get(0), pose.get(1), pose.get(2)]))
    }
    return table(rows)
  }
  let car = @mctl.ReedsShepp::new(turning_radius=radius, extent=2.5) catch {
    _ => return "ERR refused"
  }
  if mode == 0 {
    let path = car.path(start~, goal~) catch { _ => return "ERR unreachable" }
    let rows : Array[String] = []
    for i in 0..<count {
      let pose = path.at(path.length * i.to_double() / (count - 1).to_double())
      rows.push(record([pose.get(0), pose.get(1), pose.get(2)]))
    }
    return table(rows)
  }
  let space : &@contract.Space = car
  let wall = Wall::{ }
  let validity : &@contract.Validity = wall
  let planner : &@contract.Planner = @mctl.RrtStar::new(
    space~,
    validity~,
    step=0.5,
    goal_bias=0.15,
    neighbourhood=1.0,
    seed=11,
  ) catch {
    _ => return "ERR refused"
  }
  let path = planner.plan(start, goal, { iterations: 260, seconds: None, }) catch {
    _ => return "ERR no route"
  }
  // Each edge of the tree is a motion the space itself joins, so the drawing walks
  // the space's own interpolation rather than the straight line between nodes. The
  // number of steps follows the arc, for the reason the collision check does: a
  // fixed count over a long arc draws chords that cut the corner the route took.
  let rows : Array[String] = []
  for i in 1..<path.length() {
    let edge = space.distance(path[i - 1], path[i])
    let mut steps = (edge / 0.05).ceil().to_int()
    if steps < 4 {
      steps = 4
    } else if steps > 24 {
      steps = 24
    }
    for k in 0..<steps {
      let pose = space.interpolate(
        path[i - 1],
        path[i],
        k.to_double() / steps.to_double(),
      )
      rows.push(record([pose.get(0), pose.get(1), pose.get(2)]))
    }
  }
  let last = path[path.length() - 1]
  rows.push(record([last.get(0), last.get(1), last.get(2)]))
  table(rows)
}

///|
/// A delta cell driven once around a circle. Each row is the platform position it
/// was asked for, the one it reached, the three upper-arm angles, the platform
/// radius, and the three elbows, so the page draws the mechanism out of what the
/// library solved.
fn delta_trace(radius : Double, height : Double, samples : Int32) -> String {
  let cell = @mctl.Delta::new(
    base_radius=0.6,
    platform_radius=0.12,
    upper_arm=0.45,
    lower_arm=1.0,
  ) catch {
    _ => return "ERR refused"
  }
  let count = if samples < 4 { 4 } else { samples }
  let rows : Array[String] = []
  let mut seed = @mctl.Vec::zeros(3)
  for i in 0..<count {
    let angle = 6.283185307179586 * i.to_double() / count.to_double()
    let target = @mctl.Vec::of([
      radius * @math.cos(angle),
      radius * @math.sin(angle),
      -height,
    ])
    let solved = cell.inverse(target, seed~) catch { _ => continue }
    seed = solved
    let reached = cell.forward(solved)
    let elbows = cell.elbows(solved)
    rows.push(
      record([
        target.get(0),
        target.get(1),
        target.get(2),
        reached.get(0),
        reached.get(1),
        reached.get(2),
        solved.get(0),
        solved.get(1),
        solved.get(2),
        0.12,
        elbows[0].get(0),
        elbows[0].get(1),
        elbows[0].get(2),
        elbows[1].get(0),
        elbows[1].get(1),
        elbows[1].get(2),
        elbows[2].get(0),
        elbows[2].get(1),
        elbows[2].get(2),
      ]),
    )
  }
  table(rows)
}
