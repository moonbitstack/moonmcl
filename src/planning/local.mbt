// The two local planners. Neither searches: both answer with a command for this
// period from what is around the vehicle now, which is what makes them cheap
// enough to run inside a control loop and myopic enough to need a global plan.

///|
/// The dynamic window approach: sample the velocities the drives can reach in one
/// period, roll each one forward, and keep the best by heading, clearance and
/// speed.
#derive(Debug)
pub(all) struct DynamicWindow {
  max_velocity : Double
  min_velocity : Double
  max_rotation : Double
  max_acceleration : Double
  max_rotation_acceleration : Double
  resolution_velocity : Double
  resolution_rotation : Double
  predict : Double
  clearance : Double
  margin : Double
  heading_weight : Double
  clearance_weight : Double
  velocity_weight : Double
}

///|
/// The preset dynamic window: a metre per second, two radians per second, and a
/// prediction of a second and a half.
pub let dynamic_window : DynamicWindow = {
  max_velocity: 1.0,
  min_velocity: 0.0,
  max_rotation: 2.0,
  max_acceleration: 0.5,
  max_rotation_acceleration: 2.0,
  resolution_velocity: 0.05,
  resolution_rotation: 0.1,
  predict: 1.5,
  clearance: 0.5,
  margin: 0.05,
  heading_weight: 1.0,
  clearance_weight: 0.5,
  velocity_weight: 0.2,
}

///|
/// The best velocity pair for the current pose towards `goal`, avoiding
/// `obstacles`, which are points.
pub fn DynamicWindow::command(
  self : DynamicWindow,
  pose~ : @linear.Vec,
  velocity~ : @linear.Vec,
  goal~ : @linear.Vec,
  obstacles~ : ArrayView[@linear.Vec],
  dt~ : Double = 0.1,
) -> @linear.Vec raise @linear.Refused {
  if pose.len() < 3 || velocity.len() < 2 || goal.len() < 2 {
    raise @linear.Refused::Dimension(got=pose.len(), want=3)
  }
  if self.resolution_velocity <= 0.0 || self.resolution_rotation <= 0.0 {
    raise @linear.Refused::Gain(got=self.resolution_velocity)
  }
  // The window: what the drives can reach from the current velocity in one
  // period, cut by the absolute limits.
  let low_velocity = if velocity.get(0) - self.max_acceleration * dt >
    self.min_velocity {
    velocity.get(0) - self.max_acceleration * dt
  } else {
    self.min_velocity
  }
  let high_velocity = if velocity.get(0) + self.max_acceleration * dt <
    self.max_velocity {
    velocity.get(0) + self.max_acceleration * dt
  } else {
    self.max_velocity
  }
  let low_rotation = velocity.get(1) - self.max_rotation_acceleration * dt
  let high_rotation = velocity.get(1) + self.max_rotation_acceleration * dt
  let mut best_velocity = 0.0
  let mut best_rotation = 0.0
  let mut best_score = -1.0e30
  let mut best_clearance = -1.0e30
  let mut v = low_velocity
  while v <= high_velocity + 1.0e-12 {
    let mut w = low_rotation
    while w <= high_rotation + 1.0e-12 {
      let bounded = if w > self.max_rotation {
        self.max_rotation
      } else if w < -self.max_rotation {
        -self.max_rotation
      } else {
        w
      }
      let (score, clearance) = self.evaluate(pose, goal, obstacles, v, bounded)
      // A candidate that would come closer than the margin is admissible only
      // when nothing else is: the fallback is the one that keeps the most room.
      if score > best_score ||
        (best_score <= -1.0e29 && clearance > best_clearance) {
        best_score = score
        best_clearance = clearance
        best_velocity = v
        best_rotation = bounded
      }
      w = w + self.resolution_rotation
    }
    v = v + self.resolution_velocity
  }
  @linear.Vec::of([best_velocity, best_rotation])
}

///|
/// The score of one candidate and the room it leaves: how straight the rolled-out
/// heading ends up, how much room is left, and how fast it goes. A candidate that
/// comes closer than the margin scores below every admissible one.
fn DynamicWindow::evaluate(
  self : DynamicWindow,
  pose : @linear.Vec,
  goal : @linear.Vec,
  obstacles : ArrayView[@linear.Vec],
  velocity : Double,
  rotation : Double,
) -> (Double, Double) {
  let mut x = pose.get(0)
  let mut y = pose.get(1)
  let mut heading = pose.get(2)
  let steps = 16
  let step = self.predict / steps.to_double()
  let mut clearance = 1.0e30
  for _ in 0..<steps {
    let next_x = x +
      velocity * @math.cos(heading + 0.5 * rotation * step) * step
    let next_y = y +
      velocity * @math.sin(heading + 0.5 * rotation * step) * step
    // The distance that matters is to the arc the vehicle sweeps, not to the
    // points it happens to be sampled at: a wall between two samples is exactly
    // the case this planner exists to handle.
    for i in 0..<obstacles.length() {
      let distance = segment_distance(
        x,
        y,
        next_x,
        next_y,
        obstacles[i].get(0),
        obstacles[i].get(1),
      )
      if distance < clearance {
        clearance = distance
      }
    }
    x = next_x
    y = next_y
    heading = heading + rotation * step
  }
  let to_goal = @math.atan2(goal.get(1) - y, goal.get(0) - x)
  let alignment = @math.cos(to_goal - heading)
  let room = if clearance < self.clearance {
    clearance / self.clearance
  } else {
    1.0
  }
  let score = if clearance < self.margin {
    -1.0e30
  } else {
    self.heading_weight * alignment +
    self.clearance_weight * room +
    self.velocity_weight * (velocity / self.max_velocity)
  }
  (score, clearance)
}

///|
/// The distance from a point to a line segment.
fn segment_distance(
  ax : Double,
  ay : Double,
  bx : Double,
  by : Double,
  px : Double,
  py : Double,
) -> Double {
  let dx = bx - ax
  let dy = by - ay
  let length_squared = dx * dx + dy * dy
  let t = if length_squared > 0.0 {
    let raw = ((px - ax) * dx + (py - ay) * dy) / length_squared
    if raw < 0.0 {
      0.0
    } else if raw > 1.0 {
      1.0
    } else {
      raw
    }
  } else {
    0.0
  }
  let cx = ax + t * dx
  let cy = ay + t * dy
  let ex = px - cx
  let ey = py - cy
  (ex * ex + ey * ey).sqrt()
}

///|
/// An artificial potential field: attraction to the goal, repulsion from the
/// obstacles.
#derive(Debug)
pub(all) struct PotentialField {
  attraction : Double
  repulsion : Double
  influence : Double
  step : Double
}

///|
/// The preset potential field.
pub let potential_field : PotentialField = {
  attraction: 1.0,
  repulsion: 1.0,
  influence: 1.0,
  step: 0.05,
}

///|
/// The velocity command for this step, as a two-dimensional motion.
pub fn PotentialField::command(
  self : PotentialField,
  position~ : @linear.Vec,
  goal~ : @linear.Vec,
  obstacles~ : ArrayView[@linear.Vec],
) -> @linear.Vec {
  let n = position.len()
  let mut force = goal.sub(position).scale(self.attraction)
  for i in 0..<obstacles.length() {
    let away = position.sub(obstacles[i])
    let distance = away.norm()
    if distance <= 0.0 || distance >= self.influence {
      continue
    }
    // The classical repulsion falls off as the inverse square of the distance and
    // grows as the obstacle gets closer than the influence radius.
    let magnitude = self.repulsion *
      (1.0 / distance - 1.0 / self.influence) /
      (distance * distance)
    force = force.add(away.scale(magnitude / distance))
  }
  let norm = force.norm()
  if norm == 0.0 {
    return @linear.Vec::zeros(n)
  }
  force.scale(self.step / norm)
}

///|
pub extend DynamicWindow with @debug.Debug::{to_repr}

///|
pub extend PotentialField with @debug.Debug::{to_repr}
