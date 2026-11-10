// Tests for the kinematics package. Each mechanism is checked against its own
// inverse, which is the property a controller depends on, and against a pose
// whose answer can be worked out by hand.

///|
fn refused(f : () -> Unit raise @linear.Refused) -> @linear.Refused? {
  try {
    f()
    None
  } catch {
    e => Some(e)
  }
}

///|
fn close(a : Double, b : Double, tolerance~ : Double = 1.0e-9) -> Bool {
  (a - b).abs() <= tolerance
}

// --- wheeled bases --------------------------------------------------------

///|
test "differential forward and inverse agree" {
  let base = @kinematics.Differential::new(wheel_radius=0.1, track=0.5)
  let wheels = @linear.Vec::of([2.0, 3.0])
  let twist = base.forward(wheels)
  // (2 + 3)/2 * 0.1 = 0.25 m/s, (3 - 2) * 0.1 / 0.5 = 0.2 rad/s.
  assert_eq(close(twist.get(0), 0.25), true)
  assert_eq(close(twist.get(1), 0.2), true)
  let back = base.inverse(twist)
  assert_eq(back.approx(wheels, tolerance=1.0e-12), true)
}

///|
test "differential advance follows the arc" {
  let base = @kinematics.Differential::new(wheel_radius=0.1, track=0.5)
  // A quarter turn at one metre per second for pi/2 seconds: one metre of arc
  // around a one-metre radius ends up one metre to the side.
  let pose = base.advance(
    pose=@linear.Vec::of([0.0, 0.0, 0.0]),
    twist=@linear.Vec::of([1.0, 1.0]),
    dt=1.5707963267948966,
  )
  assert_eq(close(pose.get(0), 1.0, tolerance=1.0e-9), true)
  assert_eq(close(pose.get(1), 1.0, tolerance=1.0e-9), true)
  assert_eq(close(pose.get(2), 1.5707963267948966, tolerance=1.0e-9), true)
}

///|
test "bicycle curvature and steering invert" {
  let bike = @kinematics.Bicycle::new(wheelbase=2.0)
  let curvature = bike.curvature(steering=0.3)
  assert_eq(close(bike.steering(curvature~), 0.3, tolerance=1.0e-12), true)
}

///|
test "ackermann turns the inner wheel further" {
  let car = @kinematics.Ackermann::new(wheelbase=2.0, track=1.0)
  let (inner, outer) = car.angles(curvature=0.5)
  assert_eq(inner > outer, true)
  assert_eq(inner > 0.0, true)
  // atan(2 / (2 - 0.5)) for the inner wheel on a two-metre radius.
  assert_eq(close(inner, 0.9272952180016122, tolerance=1.0e-9), true)
}

///|
test "mecanum forward and inverse agree" {
  let base = @kinematics.Mecanum::new(
    wheel_radius=0.1,
    half_length=0.3,
    half_width=0.25,
  )
  let twist = @linear.Vec::of([0.5, -0.2, 0.3])
  let wheels = base.inverse(twist)
  let back = base.forward(wheels)
  assert_eq(back.approx(twist, tolerance=1.0e-12), true)
}

///|
test "omnidirectional with three wheels inverts exactly" {
  let base = @kinematics.Omnidirectional::new(wheel_radius=0.1, radius=0.3, angles=[
    0.0, 2.0943951023931953, 4.1887902047863905,
  ])
  let twist = @linear.Vec::of([0.4, 0.1, -0.2])
  let wheels = base.inverse(twist)
  let back = base.forward(wheels)
  assert_eq(back.approx(twist, tolerance=1.0e-9), true)
}

// --- serial arms ----------------------------------------------------------

///|
test "planar arm reaches a pose through its inverse" {
  let arm = @kinematics.PlanarArm::new(lengths=[1.0, 1.0])
  let target = @linear.Vec::of([1.2, 0.7])
  let joint = arm.inverse(target)
  let reached = arm.forward(joint)
  assert_eq(reached.approx(target, tolerance=1.0e-6), true)
}

///|
test "planar jacobian matches a finite difference" {
  let arm = @kinematics.PlanarArm::new(lengths=[1.0, 0.8, 0.5])
  let joint = @linear.Vec::of([0.3, -0.4, 0.9])
  let analytic = arm.jacobian_analytic(joint)
  let bump = 1.0e-7
  for k in 0..<3 {
    let shifted = joint.clone()
    shifted.set(k, shifted.get(k) + bump)
    let plus = arm.forward(shifted)
    let base = arm.forward(joint)
    let difference = plus.sub(base).scale(1.0 / bump)
    assert_eq(
      close(difference.get(0), analytic.get(0, k), tolerance=1.0e-5),
      true,
    )
    assert_eq(
      close(difference.get(1), analytic.get(1, k), tolerance=1.0e-5),
      true,
    )
  }
}

///|
test "planar arm refuses a pose it cannot reach" {
  let arm = @kinematics.PlanarArm::new(lengths=[1.0, 1.0])
  match
    refused(fn() raise @linear.Refused {
      ignore(arm.inverse(@linear.Vec::of([5.0, 0.0])))
    }) {
    Some(_) => ()
    None => abort("expected a refusal")
  }
}

///|
test "scara forward and inverse agree" {
  let arm = @kinematics.Scara::new(upper_arm=1.0, forearm=0.8, vertical=0.5)
  let target = @linear.Vec::of([1.2, 0.4, 0.2])
  let joint = arm.inverse(target)
  let reached = arm.forward(joint)
  assert_eq(reached.approx(target, tolerance=1.0e-9), true)
}

// --- parallel and gantry structures ---------------------------------------

///|
test "delta forward and inverse agree" {
  let cell = @kinematics.Delta::new(
    base_radius=0.5,
    platform_radius=0.1,
    upper_arm=0.4,
    lower_arm=0.8,
  )
  let target = @linear.Vec::of([0.05, -0.03, -1.0])
  let joint = cell.inverse(target)
  let reached = cell.forward(joint)
  assert_eq(reached.approx(target, tolerance=1.0e-9), true)
}

///|
test "delta refuses a pose beyond its reach" {
  let cell = @kinematics.Delta::new(
    base_radius=0.5,
    platform_radius=0.1,
    upper_arm=0.4,
    lower_arm=0.8,
  )
  match
    refused(fn() raise @linear.Refused {
      ignore(cell.inverse(@linear.Vec::of([0.0, 0.0, -5.0])))
    }) {
    Some(_) => ()
    None => abort("expected a refusal")
  }
}

///|
test "gantry separates translation from tilt" {
  let gantry = @kinematics.Gantry::new(track=2.0)
  let joint = @linear.Vec::of([1.0, 3.0, 3.2, -0.5])
  let task = gantry.forward(joint)
  // The middle of the beam sits at 3.1 and the ends differ by 0.1 over 2 metres.
  assert_eq(close(task.get(1), 3.1), true)
  assert_eq(close(task.get(3), 0.1), true)
  let back = gantry.inverse(task)
  assert_eq(back.approx(joint, tolerance=1.0e-12), true)
}

///|
test "a mechanism written outside the library works with the contract" {
  // A prismatic single axis: the joint value is the position.
  let axis : &@contract.Kinematics = Prismatic::{ length: 1.0, }
  let joint = axis.inverse(@linear.Vec::of([0.4]))
  assert_eq(close(joint.get(0), 0.4), true)
  assert_eq(close(axis.forward(joint).get(0), 0.4), true)
}

///|
/// A one-axis slide, to show that the contract takes a mechanism from anywhere.
pub struct Prismatic {
  length : Double
}

///|
pub impl @contract.Kinematics for Prismatic with fn forward(self, q) {
  ignore(self)
  @linear.Vec::of([q.get(0)])
}

///|
pub impl @contract.Kinematics for Prismatic with fn inverse(self, x, seed?) {
  ignore(seed)
  if x.get(0) < 0.0 || x.get(0) > self.length {
    raise @linear.Refused::Unreachable
  }
  @linear.Vec::of([x.get(0)])
}

///|
pub extend Prismatic with @contract.Kinematics::{forward, inverse, jacobian}
