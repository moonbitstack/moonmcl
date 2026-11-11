// The Dubins car: forward only, a fixed turning radius. Its shortest path
// between two poses is one of six words of arcs and lines, and because the space
// implements `Space`, a sampling planner can plan in the car's own geometry
// without knowing any of that.

///|
/// The half turn, for folding an angle into `[-pi, pi)`.
let half_turn : Double = 3.141592653589793

///|
/// The full turn.
let full_turn : Double = 6.283185307179586

///|
/// Fold an arc length into `[0, 2 pi)`, which is the range a segment of a Dubins
/// word has to sit in: a negative one would mean turning the other way, which is
/// a different word.
fn mod_length(angle : Double) -> Double {
  let mut value = angle
  while value >= full_turn {
    value = value - full_turn
  }
  while value < 0.0 {
    value = value + full_turn
  }
  value
}

///|
/// Fold an angle into `[-pi, pi)`.
fn mod_two_pi(angle : Double) -> Double {
  let mut value = angle
  while value >= half_turn {
    value = value - full_turn
  }
  while value < -half_turn {
    value = value + full_turn
  }
  value
}

///|
/// One of the six Dubins words: three pieces, each a left arc, a right arc or a
/// straight line.
#derive(Debug)
pub(all) enum DubinsWord {
  ///
  /// Left, straight, left.
  Lsl
  ///
  /// Left, straight, right.
  Lsr
  ///
  /// Right, straight, left.
  Rsl
  ///
  /// Right, straight, right.
  Rsr
  ///
  /// Right, left, right.
  Rlr
  ///
  /// Left, right, left.
  Lrl
}

///|
/// The Dubins path between two poses: the word and the length of each of its
/// three pieces.
#derive(Debug)
pub(all) struct DubinsPath {
  word : DubinsWord
  lengths : (Double, Double, Double)
  length : Double
  origin : @linear.Vec
  radius : Double
}

///|
/// A Dubins car.
#derive(Debug)
pub(all) struct Dubins {
  turning_radius : Double
  extent : Double
}

///|
/// A Dubins car with the given radius. `extent` is the half width of the box the
/// space samples from.
pub fn Dubins::new(
  turning_radius~ : Double,
  extent~ : Double = 1.0,
) -> Dubins raise @linear.Refused {
  if turning_radius <= 0.0 {
    raise @linear.Refused::Gain(got=turning_radius)
  }
  if extent <= 0.0 {
    raise @linear.Refused::Gain(got=extent)
  }
  { turning_radius, extent, }
}

///|
/// The three piece lengths of each word, in turning radii, or `None` when the
/// word cannot join the two poses.
fn word_lengths(
  word : DubinsWord,
  alpha : Double,
  beta : Double,
  distance : Double,
) -> (Double, Double, Double)? {
  let sa = @math.sin(alpha)
  let sb = @math.sin(beta)
  let ca = @math.cos(alpha)
  let cb = @math.cos(beta)
  let cab = @math.cos(alpha - beta)
  match word {
    Lsl => {
      let squared = 2.0 +
        distance * distance -
        2.0 * cab +
        2.0 * distance * (sa - sb)
      if squared < 0.0 {
        return None
      }
      let tmp = @math.atan2(cb - ca, distance + sa - sb)
      Some((mod_length(-alpha + tmp), squared.sqrt(), mod_length(beta - tmp)))
    }
    Rsr => {
      let squared = 2.0 +
        distance * distance -
        2.0 * cab +
        2.0 * distance * (sb - sa)
      if squared < 0.0 {
        return None
      }
      let tmp = @math.atan2(ca - cb, distance - sa + sb)
      Some((mod_length(alpha - tmp), squared.sqrt(), mod_length(-beta + tmp)))
    }
    Lsr => {
      let squared = -2.0 +
        distance * distance +
        2.0 * cab +
        2.0 * distance * (sa + sb)
      if squared < 0.0 {
        return None
      }
      let p = squared.sqrt()
      let tmp = @math.atan2(-ca - cb, distance + sa + sb) - @math.atan2(-2.0, p)
      Some((mod_length(-alpha + tmp), p, mod_length(-mod_two_pi(beta) + tmp)))
    }
    Rsl => {
      let squared = distance * distance -
        2.0 +
        2.0 * cab -
        2.0 * distance * (sa + sb)
      if squared < 0.0 {
        return None
      }
      let p = squared.sqrt()
      let tmp = @math.atan2(ca + cb, distance - sa - sb) - @math.atan2(2.0, p)
      Some((mod_length(alpha - tmp), p, mod_length(beta - tmp)))
    }
    Rlr => {
      let tmp = (
          6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (sa - sb)
        ) /
        8.0
      if tmp.abs() > 1.0 {
        return None
      }
      let phi = @math.atan2(ca - cb, distance - sa + sb)
      let p = mod_length(full_turn - @math.acos(tmp))
      let t = mod_length(alpha - phi + p / 2.0)
      Some((t, p, mod_length(alpha - beta - t + p)))
    }
    Lrl => {
      let tmp = (
          6.0 - distance * distance + 2.0 * cab + 2.0 * distance * (sb - sa)
        ) /
        8.0
      if tmp.abs() > 1.0 {
        return None
      }
      let phi = @math.atan2(ca - cb, distance + sa - sb)
      let p = mod_length(full_turn - @math.acos(tmp))
      let t = mod_length(-alpha - phi + p / 2.0)
      Some((t, p, mod_length(beta - alpha - t + p)))
    }
  }
}

///|
/// Every word that joins the two poses, in word order.
///
/// A car whose words are all drawn is a car the reader can compare; the callers
/// that only want to drive one take `path`.
pub fn Dubins::paths(
  self : Dubins,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> Array[DubinsPath] raise @linear.Refused {
  if start.len() < 3 || goal.len() < 3 {
    raise @linear.Refused::Dimension(got=start.len(), want=3)
  }
  let radius = self.turning_radius
  // The words are written for a car at the origin facing along the x axis, so the
  // goal goes into that frame first; the path then runs back out through the
  // start pose's heading.
  let offset_x = goal.get(0) - start.get(0)
  let offset_y = goal.get(1) - start.get(1)
  let cosine = @math.cos(start.get(2))
  let sine = @math.sin(start.get(2))
  let dx = (offset_x * cosine + offset_y * sine) / radius
  let dy = (-offset_x * sine + offset_y * cosine) / radius
  let distance = (dx * dx + dy * dy).sqrt()
  let heading = @math.atan2(dy, dx)
  let alpha = mod_two_pi(-heading)
  let beta = mod_two_pi(goal.get(2) - start.get(2) - heading)
  let words = [
    DubinsWord::Lsl,
    DubinsWord::Rsr,
    DubinsWord::Lsr,
    DubinsWord::Rsl,
    DubinsWord::Rlr,
    DubinsWord::Lrl,
  ]
  let paths : Array[DubinsPath] = []
  for i in 0..<6 {
    match word_lengths(words[i], alpha, beta, distance) {
      None => ()
      Some(lengths) =>
        match lengths {
          (first, second, third) =>
            paths.push({
              word: words[i],
              lengths: (first * radius, second * radius, third * radius),
              length: (first + second + third) * radius,
              origin: start,
              radius,
            })
        }
    }
  }
  paths
}

///|
/// The shortest Dubins path between two `(x, y, heading)` poses.
pub fn Dubins::path(
  self : Dubins,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> DubinsPath raise @linear.Refused {
  if start.len() < 3 || goal.len() < 3 {
    raise @linear.Refused::Dimension(got=start.len(), want=3)
  }
  let radius = self.turning_radius
  // The words are written for a car at the origin facing along the x axis, so the
  // goal goes into that frame first; the path then runs back out through the
  // start pose's heading.
  let offset_x = goal.get(0) - start.get(0)
  let offset_y = goal.get(1) - start.get(1)
  let cosine = @math.cos(start.get(2))
  let sine = @math.sin(start.get(2))
  let dx = (offset_x * cosine + offset_y * sine) / radius
  let dy = (-offset_x * sine + offset_y * cosine) / radius
  let distance = (dx * dx + dy * dy).sqrt()
  let heading = @math.atan2(dy, dx)
  let alpha = mod_two_pi(-heading)
  let beta = mod_two_pi(goal.get(2) - start.get(2) - heading)
  let words = [
    DubinsWord::Lsl,
    DubinsWord::Rsr,
    DubinsWord::Lsr,
    DubinsWord::Rsl,
    DubinsWord::Rlr,
    DubinsWord::Lrl,
  ]
  let mut best : DubinsPath? = None
  for i in 0..<6 {
    match word_lengths(words[i], alpha, beta, distance) {
      None => ()
      Some(lengths) =>
        match lengths {
          (first, second, third) => {
            // The word lengths are in turning radii and a path's length is in
            // metres, so the comparison is made in metres.
            let total = (first + second + third) * radius
            match best {
              Some(current) =>
                if total < current.length {
                  best = Some({
                    word: words[i],
                    lengths: (first * radius, second * radius, third * radius),
                    length: total,
                    origin: start,
                    radius,
                  })
                }
              None =>
                best = Some({
                  word: words[i],
                  lengths: (first * radius, second * radius, third * radius),
                  length: total,
                  origin: start,
                  radius,
                })
            }
          }
        }
    }
  }
  match best {
    Some(path) => path
    None => raise @linear.Refused::Unreachable
  }
}

///|
/// The pose at `distance` travelled along the path.
pub fn DubinsPath::at(self : DubinsPath, distance : Double) -> @linear.Vec {
  let (first, second, third) = self.lengths
  let radius = self.radius
  // Which piece the distance falls in, and how far into it.
  let mut remaining = distance
  let pieces = [
    (piece_of(self.word, 0), first),
    (piece_of(self.word, 1), second),
    (piece_of(self.word, 2), third),
  ]
  let mut x = self.origin.get(0)
  let mut y = self.origin.get(1)
  let mut heading = self.origin.get(2)
  for i in 0..<3 {
    let (turn, length) = pieces[i]
    let travelled = if remaining > length { length } else { remaining }
    if turn != 0.0 && radius > 0.0 {
      let angle = travelled / radius * turn
      // The centre of the arc sits to the side the turn goes.
      let centre_bearing = heading + turn * half_turn / 2.0 * 2.0
      ignore(centre_bearing)
      let side = if turn > 0.0 { 1.0 } else { -1.0 }
      let cx = x - side * radius * @math.sin(heading)
      let cy = y + side * radius * @math.cos(heading)
      let next = heading + angle
      x = cx + side * radius * @math.sin(next)
      y = cy - side * radius * @math.cos(next)
      heading = next
    } else {
      x = x + travelled * @math.cos(heading)
      y = y + travelled * @math.sin(heading)
    }
    remaining = remaining - travelled
    if remaining <= 0.0 {
      break
    }
  }
  @linear.Vec::of([x, y, mod_two_pi(heading)])
}

///|
/// Which way piece `index` of a word turns: `1` left, `-1` right, `0` straight.
fn piece_of(word : DubinsWord, index : Int32) -> Double {
  match word {
    Lsl => if index == 1 { 0.0 } else { 1.0 }
    Lsr => if index == 1 { 0.0 } else if index == 0 { 1.0 } else { -1.0 }
    Rsl => if index == 1 { 0.0 } else if index == 0 { -1.0 } else { 1.0 }
    Rsr => if index == 1 { 0.0 } else { -1.0 }
    Rlr => if index == 1 { 1.0 } else { -1.0 }
    Lrl => if index == 1 { -1.0 } else { 1.0 }
  }
}

///|
pub impl @contract.Space for Dubins with fn sample(self, rng) {
  let x = (@rand.random(rng) * 2.0 - 1.0) * self.extent
  let y = (@rand.random(rng) * 2.0 - 1.0) * self.extent
  let heading = @rand.random(rng) * full_turn - half_turn
  @linear.Vec::of([x, y, heading])
}

///|
pub impl @contract.Space for Dubins with fn distance(self, a, b) {
  // A pose that is not three numbers has no Dubins path; the straight-line
  // distance keeps the planner moving rather than failing the whole search.
  self.path(start=a, goal=b).length catch {
    _ => {
      let dx = b.get(0) - a.get(0)
      let dy = b.get(1) - a.get(1)
      (dx * dx + dy * dy).sqrt()
    }
  }
}

///|
pub impl @contract.Space for Dubins with fn interpolate(self, a, b, t) {
  try {
    let path = self.path(start=a, goal=b)
    path.at(path.length * t)
  } catch {
    _ =>
      @linear.Vec::of([
        a.get(0) + t * (b.get(0) - a.get(0)),
        a.get(1) + t * (b.get(1) - a.get(1)),
        a.get(2) + t * (b.get(2) - a.get(2)),
      ])
  }
}

///|
pub extend Dubins with @contract.Space::{sample, distance, interpolate}

///|
pub extend DubinsWord with @debug.Debug::{to_repr}

///|
pub extend DubinsPath with @debug.Debug::{to_repr}

///|
pub extend Dubins with @debug.Debug::{to_repr}
