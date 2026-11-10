///|
/// The Rauch-Tung-Striebel smoother: the forward filter's estimates refined by a
/// backward pass over the same record.
///
/// `states` and `covariances` are the filtered estimates, `transitions` the
/// state matrices between consecutive steps, and `process` the process
/// covariance. The result is the smoothed state and covariance at every step,
/// whose covariances are never larger than the filtered ones.
pub fn rts(
  states~ : Array[@linear.Vec],
  covariances~ : Array[@linear.Mat],
  transitions~ : Array[@linear.Mat],
  process~ : @linear.Mat,
) -> (Array[@linear.Vec], Array[@linear.Mat]) raise @linear.Refused {
  let steps = states.length()
  if steps == 0 {
    raise @linear.Refused::Empty
  }
  if covariances.length() != steps {
    raise @linear.Refused::Dimension(got=covariances.length(), want=steps)
  }
  if steps > 1 && transitions.length() != steps - 1 {
    raise @linear.Refused::Dimension(got=transitions.length(), want=steps - 1)
  }
  let smoothed_states = Array::make(steps, states[0])
  let smoothed_covariances = Array::make(steps, covariances[0])
  for k in 0..<steps {
    smoothed_states[k] = states[k].clone()
    smoothed_covariances[k] = covariances[k]
  }
  for k in (steps - 2)>=..0 {
    let transition = transitions[k]
    let predicted_state = transition.apply(states[k])
    let predicted_covariance = transition
      .mul(covariances[k])
      .mul(transition.transpose())
      .add(process)
    // c = p f' p_pred^-1, computed as (p_pred^-1 (f p))'.
    let gain = @linear.solve(
      predicted_covariance,
      transition.mul(covariances[k]),
    ).transpose()
    smoothed_states[k] = states[k].add(
      gain.apply(smoothed_states[k + 1].sub(predicted_state)),
    )
    smoothed_covariances[k] = covariances[k].add(
      gain
      .mul(smoothed_covariances[k + 1].sub(predicted_covariance))
      .mul(gain.transpose()),
    )
  }
  (smoothed_states, smoothed_covariances)
}
