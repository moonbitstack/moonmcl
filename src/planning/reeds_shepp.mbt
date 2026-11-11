// The Reeds-Shepp car: forward and reverse, a fixed turning radius, and a path
// that may stop and back up. Where the Dubins car has six words the reversable
// one has forty-eight, because every word also has a mirrored and a reversed
// form.
//
// The formulas follow the published table: they are written for a unit radius
// with the start at the origin and the heading along the x axis, and each returns
// the arc length of its curved segments and the length of its straight one. A
// family is a set of formulas, a variant is the frame it runs in, and the two
// together name a word.

///|
/// The half turn.
let rs_half_turn : Double = 3.141592653589793

///|
/// A full turn.
let rs_full_turn : Double = 6.283185307179586

///|
/// One straight or curved piece of a Reeds-Shepp path.
#derive(Debug)
pub(all) struct ReedsSheppSegment {
  ///
  /// `1` for a left arc, `-1` for a right one, `0` for a straight.
  turn : Double
  ///
  /// `1` when the car drives forward along the piece, `-1` when it backs up.
  forward : Double
  ///
  /// The length of the piece, in metres.
  length : Double
}

///|
/// A Reeds-Shepp path: the word it spells, its pieces, and the pose it starts
/// from.
#derive(Debug)
pub(all) struct ReedsSheppPath {
  ///
  /// The word the path spells.
  word : ReedsSheppWord
  ///
  /// The pieces, in the order the car drives them.
  segments : Array[ReedsSheppSegment]
  ///
  /// The total length, in metres.
  length : Double
  origin : @linear.Vec
  radius : Double
}

///|
/// A Reeds-Shepp car.
#derive(Debug)
pub(all) struct ReedsShepp {
  turning_radius : Double
  extent : Double
}

///|
/// A Reeds-Shepp car with the given radius. `extent` is the half width of the box
/// the space samples from.
pub fn ReedsShepp::new(
  turning_radius~ : Double,
  extent~ : Double = 1.0,
) -> ReedsShepp raise @linear.Refused {
  if turning_radius <= 0.0 {
    raise @linear.Refused::Gain(got=turning_radius)
  }
  if extent <= 0.0 {
    raise @linear.Refused::Gain(got=extent)
  }
  { turning_radius, extent, }
}

///|
/// Fold an angle into `(-pi, pi]`.
fn rs_wrap(angle : Double) -> Double {
  let mut value = angle
  while value > rs_half_turn {
    value = value - rs_full_turn
  }
  while value < -rs_half_turn {
    value = value + rs_full_turn
  }
  value
}

///|
/// The polar coordinates of a point in the plane.
fn rs_polar(x : Double, y : Double) -> (Double, Double) {
  ((x * x + y * y).sqrt(), @math.atan2(y, x))
}

///|
/// A parameter set is valid when every piece has a length at or above zero; a
/// negative one would be the same word driven the other way, which is a
/// different word. The upper bound also rejects what `acos` and `asin` return
/// outside their domain, whose comparisons are false for a NaN and true for an
/// infinity.
fn rs_valid(params : (Double, Double, Double)) -> Bool {
  let (t, u, v) = params
  t >= 0.0 && u >= 0.0 && v >= 0.0 && t < 1.0e6 && u < 1.0e6 && v < 1.0e6
}

///|
/// The goal in the frame of the left circle the start pose sits on: the circle's
/// centre is at `(0, 1)` in that frame, which is the `- 1` the formulas carry.
fn rs_left_frame(x : Double, y : Double, phi : Double) -> (Double, Double) {
  (x - @math.sin(phi), y + @math.cos(phi) - 1.0)
}

///|
/// The goal in the frame of the right circle the start pose sits on, whose centre
/// is at `(0, -1)`.
fn rs_right_frame(x : Double, y : Double, phi : Double) -> (Double, Double) {
  (x + @math.sin(phi), y - @math.cos(phi) - 1.0)
}

///|
/// Formula 8.1: curve, straight, curve, the two arcs turning the same way.
fn rs_csca(x : Double, y : Double, phi : Double) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  let (u, t) = rs_polar(a, b)
  let v = rs_wrap(phi - t)
  Some((t, u, v))
}

///|
/// Formula 8.2: curve, straight, curve, the two arcs turning opposite ways.
fn rs_cscb(x : Double, y : Double, phi : Double) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 2.0 {
    return None
  }
  let u = (r * r - 4.0).sqrt()
  let alpha = @math.atan2(2.0, u)
  let t = rs_wrap(theta + alpha)
  Some((t, u, rs_wrap(t - phi)))
}

///|
/// Formula 8.3: three arcs, a cusp between each pair.
fn rs_c_c_c(x : Double, y : Double, phi : Double) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  if a.abs() < 1.0e-12 && b.abs() < 1.0e-12 {
    return None
  }
  let (r, theta) = rs_polar(a, b)
  if r >= 4.0 {
    return None
  }
  let alpha = @math.acos(r / 4.0)
  let t = rs_wrap(rs_half_turn / 2.0 + alpha + theta)
  let u = rs_wrap(rs_half_turn - 2.0 * alpha)
  Some((t, u, rs_wrap(phi - t - u)))
}

///|
/// Formula 8.4, first case: curve with a cusp, then two arcs the same way.
fn rs_c_cc(x : Double, y : Double, phi : Double) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  if a.abs() < 1.0e-12 && b.abs() < 1.0e-12 {
    return None
  }
  let (r, theta) = rs_polar(a, b)
  if r > 4.0 {
    return None
  }
  let alpha = @math.acos(r / 4.0)
  let t = rs_wrap(rs_half_turn / 2.0 + alpha + theta)
  let u = rs_wrap(rs_half_turn - 2.0 * alpha)
  Some((t, u, rs_wrap(t + u - phi)))
}

///|
/// Formula 8.4, second case: two arcs the same way, then a curve.
fn rs_cc_c(x : Double, y : Double, phi : Double) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  if a.abs() < 1.0e-12 && b.abs() < 1.0e-12 {
    return None
  }
  let (r, theta) = rs_polar(a, b)
  if r > 4.0 {
    return None
  }
  let u = @math.acos((8.0 - r * r) / 8.0)
  let mut sin_u = @math.sin(u)
  if sin_u.abs() < 0.001 {
    sin_u = 0.0
  }
  if sin_u.abs() < 0.001 && r.abs() < 0.001 {
    return None
  }
  let alpha = @math.asin(2.0 * sin_u / r)
  let t = rs_wrap(rs_half_turn / 2.0 - alpha + theta)
  Some((t, u, rs_wrap(t - u - phi)))
}

///|
/// Formula 8.7: two arcs forward and two reversed.
fn rs_ccu_cuc(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  if a.abs() < 1.0e-12 && b.abs() < 1.0e-12 {
    return None
  }
  let (r, theta) = rs_polar(a, b)
  if r > 4.0 {
    return None
  }
  let alpha = if r > 2.0 {
    @math.acos((r / 2.0 - 1.0) / 2.0)
  } else {
    @math.acos((r / 2.0 + 1.0) / 2.0)
  }
  let t = if r > 2.0 {
    rs_wrap(rs_half_turn / 2.0 + theta - alpha)
  } else {
    rs_wrap(rs_half_turn / 2.0 + theta + alpha)
  }
  let u = if r > 2.0 { rs_wrap(rs_half_turn - alpha) } else { rs_wrap(alpha) }
  Some((t, u, rs_wrap(phi - t + 2.0 * u)))
}

///|
/// Formula 8.8: a curve, two arcs one way and one the other, then a curve.
fn rs_c_cucu_c(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  if a.abs() < 1.0e-12 && b.abs() < 1.0e-12 {
    return None
  }
  let (r, theta) = rs_polar(a, b)
  if r > 6.0 {
    return None
  }
  let ratio = (5.0 - r * r / 4.0) / 4.0
  if ratio < 0.0 || ratio > 1.0 {
    return None
  }
  let u = @math.acos(ratio)
  let alpha = @math.asin(2.0 * @math.sin(u) / r)
  let t = rs_wrap(rs_half_turn / 2.0 + theta + alpha)
  Some((t, u, rs_wrap(t - phi)))
}

///|
/// Formula 8.9, first case: curve, cusp, a half turn, straight, curve.
fn rs_c_c2sca(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 2.0 {
    return None
  }
  let u = (r * r - 4.0).sqrt() - 2.0
  if u < 0.0 {
    return None
  }
  let alpha = @math.atan2(2.0, u + 2.0)
  let t = rs_wrap(rs_half_turn / 2.0 + theta + alpha)
  Some((t, u, rs_wrap(t + rs_half_turn / 2.0 - phi)))
}

///|
/// Formula 8.9, second case: curve, cusp, a half turn, straight, curve the other
/// way.
fn rs_c_c2scb(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 2.0 {
    return None
  }
  let t = rs_wrap(rs_half_turn / 2.0 + theta)
  let u = r - 2.0
  Some((t, u, rs_wrap(phi - t - rs_half_turn / 2.0)))
}

///|
/// Formula 8.9, third case: curve, straight, a half turn, curve with a cusp.
fn rs_csc2_ca(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_left_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 2.0 {
    return None
  }
  let u = (r * r - 4.0).sqrt() - 2.0
  if u < 0.0 {
    return None
  }
  let alpha = @math.atan2(u + 2.0, 2.0)
  let t = rs_wrap(rs_half_turn / 2.0 + theta - alpha)
  Some((t, u, rs_wrap(t - rs_half_turn / 2.0 - phi)))
}

///|
/// Formula 8.10, second case: curve, straight, a half turn, curve with a cusp
/// the other way.
fn rs_csc2_cb(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 2.0 {
    return None
  }
  let t = rs_wrap(theta)
  let u = r - 2.0
  Some((t, u, rs_wrap(-t - rs_half_turn / 2.0 + phi)))
}

///|
/// Formula 8.11: curve, cusp, a half turn, straight, a half turn, curve.
fn rs_c_c2sc2_c(
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let (a, b) = rs_right_frame(x, y, phi)
  let (r, theta) = rs_polar(a, b)
  if r < 4.0 {
    return None
  }
  let u = (r * r - 4.0).sqrt() - 4.0
  if u < 0.0 {
    return None
  }
  let alpha = @math.atan2(2.0, u + 4.0)
  let t = rs_wrap(rs_half_turn / 2.0 + theta + alpha)
  Some((t, u, rs_wrap(t - phi)))
}

///|
/// The formula of a family, in the frame it is given in.
fn rs_formula(
  family : Int32,
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let found = match family {
    0 => rs_csca(x, y, phi)
    1 => rs_cscb(x, y, phi)
    2 => rs_c_c_c(x, y, phi)
    3 => rs_c_cc(x, y, phi)
    4 => rs_cc_c(x, y, phi)
    5 => rs_ccu_cuc(x, y, phi)
    6 => rs_c_cucu_c(x, y, phi)
    7 => rs_c_c2sca(x, y, phi)
    8 => rs_c_c2scb(x, y, phi)
    9 => rs_csc2_ca(x, y, phi)
    10 => rs_csc2_cb(x, y, phi)
    _ => rs_c_c2sc2_c(x, y, phi)
  }
  match found {
    Some(params) => if rs_valid(params) { Some(params) } else { None }
    None => None
  }
}

///|
/// The parameters of a word in the frame its variant names: the pose is
/// reflected across the axis of travel for the odd variants and the whole
/// motion reversed for the upper two.
fn rs_parameters(
  word : ReedsSheppWord,
  x : Double,
  y : Double,
  phi : Double,
) -> (Double, Double, Double)? {
  let code = word.code()
  let family = code / 4
  let variant = code % 4
  let reflected = variant % 2 == 1
  let reversed = variant >= 2
  // The four frames a family runs in: as given, mirrored across the axis of
  // travel, reversed, and both.
  let fx = if reversed { -x } else { x }
  let fy = if reflected { -y } else { y }
  let fphi = if reflected != reversed { -phi } else { phi }
  match rs_formula(family, fx, fy, fphi) {
    None => None
    // Every parameter is a distance travelled, so a reversed word keeps the
    // magnitudes the formula returned and carries the reversal in its segment
    // directions.
    Some(params) => Some(params)
  }
}

///|
/// The order of the parameters along the segments of a family: `0` is the first
/// arc, `1` the middle one, `2` the last, and `3` a half turn the table fixes.
fn rs_layout(family : Int32) -> Array[Int32] {
  match family {
    5 | 6 => [0, 1, 1, 2]
    7 | 8 => [0, 3, 1, 2]
    9 | 10 => [0, 1, 3, 2]
    11 => [0, 3, 1, 3, 2]
    _ => [0, 1, 2]
  }
}

///|
/// The pieces of a word: a turn (`1` left, `-1` right, `0` straight) and the
/// direction the car drives it in (`1` forward, `-1` backing up), piece by piece.
///
/// The forty-eight rows are the published table read out in code order: a mirrored
/// or reversed word is not its turns reversed, because the cusp between two arcs
/// keeps its own sense.
fn rs_shape(code : Int32) -> Array[(Double, Double)] {
  match code {
    0 => [(1.0, 1.0), (0.0, 1.0), (1.0, 1.0)]
    1 => [(-1.0, 1.0), (0.0, 1.0), (-1.0, 1.0)]
    2 => [(1.0, -1.0), (0.0, -1.0), (1.0, -1.0)]
    3 => [(-1.0, -1.0), (0.0, -1.0), (-1.0, -1.0)]
    4 => [(1.0, 1.0), (0.0, 1.0), (-1.0, 1.0)]
    5 => [(-1.0, 1.0), (0.0, 1.0), (1.0, 1.0)]
    6 => [(1.0, -1.0), (0.0, -1.0), (-1.0, -1.0)]
    7 => [(-1.0, -1.0), (0.0, -1.0), (1.0, -1.0)]
    8 => [(1.0, 1.0), (-1.0, -1.0), (1.0, 1.0)]
    9 => [(-1.0, 1.0), (1.0, -1.0), (-1.0, 1.0)]
    10 => [(1.0, -1.0), (-1.0, 1.0), (1.0, -1.0)]
    11 => [(-1.0, -1.0), (1.0, 1.0), (-1.0, -1.0)]
    12 => [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
    13 => [(-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]
    14 => [(1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]
    15 => [(-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    16 => [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    17 => [(-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0)]
    18 => [(1.0, -1.0), (-1.0, -1.0), (1.0, 1.0)]
    19 => [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    20 => [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)]
    21 => [(-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
    22 => [(1.0, -1.0), (-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
    23 => [(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)]
    24 => [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    25 => [(-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (1.0, 1.0)]
    26 => [(1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (-1.0, -1.0)]
    27 => [(-1.0, -1.0), (1.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    28 => [(1.0, 1.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0)]
    29 => [(-1.0, 1.0), (1.0, -1.0), (0.0, -1.0), (-1.0, -1.0)]
    30 => [(1.0, -1.0), (-1.0, 1.0), (0.0, 1.0), (1.0, 1.0)]
    31 => [(-1.0, -1.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0)]
    32 => [(1.0, 1.0), (-1.0, -1.0), (0.0, -1.0), (-1.0, -1.0)]
    33 => [(-1.0, 1.0), (1.0, -1.0), (0.0, -1.0), (1.0, -1.0)]
    34 => [(1.0, -1.0), (-1.0, 1.0), (0.0, 1.0), (-1.0, 1.0)]
    35 => [(-1.0, -1.0), (1.0, 1.0), (0.0, 1.0), (1.0, 1.0)]
    36 => [(1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    37 => [(-1.0, 1.0), (0.0, 1.0), (1.0, 1.0), (-1.0, -1.0)]
    38 => [(1.0, -1.0), (0.0, -1.0), (-1.0, -1.0), (1.0, 1.0)]
    39 => [(-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    40 => [(1.0, 1.0), (0.0, 1.0), (1.0, 1.0), (-1.0, -1.0)]
    41 => [(-1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    42 => [(1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    43 => [(-1.0, -1.0), (0.0, -1.0), (-1.0, -1.0), (1.0, 1.0)]
    44 => [(1.0, 1.0), (-1.0, -1.0), (0.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]
    45 => [(-1.0, 1.0), (1.0, -1.0), (0.0, -1.0), (-1.0, -1.0), (1.0, 1.0)]
    46 => [(1.0, -1.0), (-1.0, 1.0), (0.0, 1.0), (1.0, 1.0), (-1.0, -1.0)]
    47 => [(-1.0, -1.0), (1.0, 1.0), (0.0, 1.0), (-1.0, 1.0), (1.0, -1.0)]
    _ => []
  }
}

///|
/// The pieces of a path, from the parameters the formulas returned.
fn rs_build(
  word : ReedsSheppWord,
  params : (Double, Double, Double),
  radius : Double,
) -> Array[ReedsSheppSegment] {
  let code = word.code()
  let family = code / 4
  let (t, u, v) = params
  let halves = rs_half_turn / 2.0
  let layout = rs_layout(family)
  let shape = rs_shape(code)
  let segments : Array[ReedsSheppSegment] = []
  for i in 0..<layout.length() {
    let length = match layout[i] {
      0 => t
      1 => u
      2 => v
      _ => halves
    }
    let (turn, forward) = shape[i]
    segments.push(ReedsSheppSegment::{ turn, forward, length: length * radius, })
  }
  segments
}

///|
/// The pose at `distance` travelled along the path: an arc turns around its own
/// centre, a straight runs along the heading, and backing up moves the car the
/// other way without changing which way it faces.
pub fn ReedsSheppPath::at(
  self : ReedsSheppPath,
  distance : Double,
) -> @linear.Vec {
  let mut remaining = distance
  let mut x = self.origin.get(0)
  let mut y = self.origin.get(1)
  let mut heading = self.origin.get(2)
  for i in 0..<self.segments.length() {
    let piece = self.segments[i]
    let travelled = if remaining > piece.length {
      piece.length
    } else {
      remaining
    }
    if piece.turn == 0.0 {
      let step = travelled * piece.forward
      x = x + step * @math.cos(heading)
      y = y + step * @math.sin(heading)
    } else {
      let signed_radius = self.radius * piece.turn
      let cx = x - signed_radius * @math.sin(heading)
      let cy = y + signed_radius * @math.cos(heading)
      let turned = heading + piece.forward * travelled / signed_radius
      x = cx + signed_radius * @math.sin(turned)
      y = cy - signed_radius * @math.cos(turned)
      heading = turned
    }
    remaining = remaining - travelled
    if remaining <= 0.0 {
      break
    }
  }
  @linear.Vec::of([x, y, rs_wrap(heading)])
}

///|
/// How many times the path changes direction.
pub fn ReedsSheppPath::cusps(self : ReedsSheppPath) -> Int32 {
  let mut count = 0
  for i in 1..<self.segments.length() {
    if self.segments[i].forward != self.segments[i - 1].forward {
      count = count + 1
    }
  }
  count
}

///|
/// Every word that joins the two poses, shortest first.
///
/// A word that cannot reach the goal returns no parameters at all, so the list
/// holds only the ones that do; the shortest of them shares its length with any
/// other candidate of the same total.
pub fn ReedsShepp::candidates(
  self : ReedsShepp,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> Array[ReedsSheppPath] raise @linear.Refused {
  if start.len() < 3 || goal.len() < 3 {
    raise @linear.Refused::Dimension(got=start.len(), want=3)
  }
  let radius = self.turning_radius
  let dx = goal.get(0) - start.get(0)
  let dy = goal.get(1) - start.get(1)
  let heading = start.get(2)
  let x = (dx * @math.cos(heading) + dy * @math.sin(heading)) / radius
  let y = (-dx * @math.sin(heading) + dy * @math.cos(heading)) / radius
  let phi = rs_wrap(goal.get(2) - heading)
  let paths : Array[ReedsSheppPath] = []
  for code in 0..<48 {
    let word = match ReedsSheppWord::of_code(code) {
      Some(found) => found
      None => continue
    }
    let params = match rs_parameters(word, x, y, phi) {
      Some(found) => found
      None => continue
    }
    let segments = rs_build(word, params, radius)
    let mut total = 0.0
    for segment in segments {
      total = total + segment.length
    }
    paths.push({ word, segments, length: total, origin: start, radius, })
  }
  paths
}

///|
/// The shortest path between two `(x, y, heading)` poses, reversing as needed.
pub fn ReedsShepp::path(
  self : ReedsShepp,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> ReedsSheppPath raise @linear.Refused {
  let paths = self.candidates(start~, goal~)
  let mut best : ReedsSheppPath? = None
  for path in paths {
    match best {
      None => best = Some(path)
      Some(current) => if path.length < current.length { best = Some(path) }
    }
  }
  match best {
    Some(path) => path
    None => raise @linear.Refused::Unreachable
  }
}

///|
pub impl @contract.Space for ReedsShepp with fn sample(self, rng) {
  let x = (@rand.random(rng) * 2.0 - 1.0) * self.extent
  let y = (@rand.random(rng) * 2.0 - 1.0) * self.extent
  let heading = @rand.random(rng) * rs_full_turn - rs_half_turn
  @linear.Vec::of([x, y, heading])
}

///|
pub impl @contract.Space for ReedsShepp with fn distance(self, a, b) {
  // A pose that is not three numbers has no path; the straight-line distance
  // keeps the planner moving rather than failing the whole search.
  self.path(start=a, goal=b).length catch {
    _ => {
      let dx = b.get(0) - a.get(0)
      let dy = b.get(1) - a.get(1)
      (dx * dx + dy * dy).sqrt()
    }
  }
}

///|
pub impl @contract.Space for ReedsShepp with fn interpolate(self, a, b, t) {
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
pub extend ReedsShepp with @contract.Space::{sample, distance, interpolate}

///|
pub extend ReedsSheppSegment with @debug.Debug::{to_repr}

///|
pub extend ReedsSheppPath with @debug.Debug::{to_repr}

///|
pub extend ReedsShepp with @debug.Debug::{to_repr}

///|
pub extend ReedsSheppWord with @debug.Debug::{to_repr}
