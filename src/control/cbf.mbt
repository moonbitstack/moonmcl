// The safety filter. A nominal controller is the one that is allowed to be fast
// and optimistic; this stage only ever moves its command towards the safe side,
// and by as little as it can.

///|
/// The knobs of the control-barrier-function filter.
///
/// `iterations` bounds the alternating projection. The filter is exact when it
/// converges, and the bound is there so a pathological gradient cannot stall a
/// control period.
#derive(Debug)
pub(all) struct CbfConfig {
  iterations : Int32
  tolerance : Double
}

///|
/// The preset the barrier filter runs with.
pub let cbf_config : CbfConfig = { iterations: 64, tolerance: 1.0e-9, }

///|
/// The safe input closest to the nominal one.
///
/// The barrier `h` is the one the caller computed for the current state;
/// `gradient` is `d h / d u`, `drift` is `d h / d t` without the input, and
/// `alpha` is the class-K gain that decides how hard the filter pushes towards a
/// larger barrier. The result satisfies
/// `drift + gradient * u + alpha * barrier >= 0` inside the box, or the closest
/// point to that set when the box and the half-space do not meet.
///
/// Dykstra's alternating projection rather than plain alternation: plain
/// alternation lands on a point of the intersection but not on the nearest one,
/// and the whole point of a safety filter is to change the command as little as
/// possible.
pub fn cbf_filter(
  nominal~ : @linear.Vec,
  gradient~ : @linear.Vec,
  drift~ : Double,
  barrier~ : Double,
  alpha~ : Double,
  lower~ : @linear.Vec,
  upper~ : @linear.Vec,
  config~ : CbfConfig = cbf_config,
) -> @linear.Vec {
  let n = nominal.len()
  let mut x = nominal.clone()
  let mut correction_box = @linear.Vec::zeros(n)
  let mut correction_half = @linear.Vec::zeros(n)
  let offset = drift + alpha * barrier
  let norm_squared = gradient.dot(gradient)
  for _ in 0..<config.iterations {
    let with_box = x.add(correction_box)
    let boxed = @linear.Vec::zeros(n)
    for i in 0..<n {
      boxed.set(i, clamp_scalar(with_box.get(i), lower.get(i), upper.get(i)))
    }
    correction_box = with_box.sub(boxed)
    let with_half = boxed.add(correction_half)
    let mut safe = with_half.clone()
    if norm_squared > 0.0 {
      let slack = gradient.dot(with_half) + offset
      if slack < 0.0 {
        safe = with_half.axpy(-slack / norm_squared, gradient)
      }
    }
    correction_half = with_half.sub(safe)
    let shift = safe.sub(x).max_abs()
    x = safe
    if shift <= config.tolerance {
      break
    }
  }
  x
}

///|
pub extend CbfConfig with @debug.Debug::{to_repr}
