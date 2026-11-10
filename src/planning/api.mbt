///|
/// A rapidly-exploring random tree.
pub(all) struct Rrt {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
}

///|
/// An RRT that extends by `step` and steers to the goal once every
/// `1 / goal_bias` samples.
pub fn Rrt::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
) -> Rrt raise @linear.Refused {
  abort("moonmcl: Rrt::new is not implemented yet")
}

///|
pub impl @contract.Planner for Rrt with fn plan(self, start, goal, budget) {
  abort("moonmcl: Rrt::plan is not implemented yet")
}

///|
pub extend Rrt with @contract.Planner::{plan}

///|
/// An RRT that rewires the tree as it grows, so the path converges on the
/// shortest one.
pub(all) struct RrtStar {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
  neighbourhood : Double
}

///|
/// An RRT* with a rewiring radius.
pub fn RrtStar::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
  neighbourhood~ : Double = 0.3,
) -> RrtStar raise @linear.Refused {
  abort("moonmcl: RrtStar::new is not implemented yet")
}

///|
pub impl @contract.Planner for RrtStar with fn plan(self, start, goal, budget) {
  abort("moonmcl: RrtStar::plan is not implemented yet")
}

///|
pub extend RrtStar with @contract.Planner::{plan}

///|
/// An RRT grown from both ends at once.
pub(all) struct RrtConnect {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
}

///|
/// A bidirectional RRT.
pub fn RrtConnect::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
) -> RrtConnect raise @linear.Refused {
  abort("moonmcl: RrtConnect::new is not implemented yet")
}

///|
pub impl @contract.Planner for RrtConnect with fn plan(
  self,
  start,
  goal,
  budget,
) {
  abort("moonmcl: RrtConnect::plan is not implemented yet")
}

///|
pub extend RrtConnect with @contract.Planner::{plan}

///|
/// An RRT* that only samples the ellipse whose foci are the start and the goal.
pub(all) struct InformedRrtStar {
  space : &@contract.Space
  validity : &@contract.Validity
  step : Double
  goal_bias : Double
  neighbourhood : Double
}

///|
/// An informed RRT* seeded from the best path found so far.
pub fn InformedRrtStar::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  step~ : Double = 0.1,
  goal_bias~ : Double = 0.05,
  neighbourhood~ : Double = 0.3,
) -> InformedRrtStar raise @linear.Refused {
  abort("moonmcl: InformedRrtStar::new is not implemented yet")
}

///|
pub impl @contract.Planner for InformedRrtStar with fn plan(
  self,
  start,
  goal,
  budget,
) {
  abort("moonmcl: InformedRrtStar::plan is not implemented yet")
}

///|
pub extend InformedRrtStar with @contract.Planner::{plan}

///|
/// A probabilistic roadmap: a graph sampled once and queried many times.
pub(all) struct Prm {
  space : &@contract.Space
  validity : &@contract.Validity
  samples : Int32
  neighbours : Int32
  connection : Double
}

///|
/// A roadmap with the given sample count and connection radius.
pub fn Prm::new(
  space~ : &@contract.Space,
  validity~ : &@contract.Validity,
  samples~ : Int32 = 500,
  neighbours~ : Int32 = 10,
  connection~ : Double = 1.0,
) -> Prm raise @linear.Refused {
  abort("moonmcl: Prm::new is not implemented yet")
}

///|
pub impl @contract.Planner for Prm with fn plan(self, start, goal, budget) {
  abort("moonmcl: Prm::plan is not implemented yet")
}

///|
pub extend Prm with @contract.Planner::{plan}

///|
/// One of the six Dubins words: three arcs and lines in a fixed order.
#derive(Debug)
pub(all) enum DubinsWord {
  Lsl
  Lsr
  Rsl
  Rsr
  Rlr
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
}

///|
/// A Dubins car: forward-only motion at a fixed turning radius.
pub(all) struct Dubins {
  turning_radius : Double
}

///|
/// A Dubins car with the given radius.
pub fn Dubins::new(turning_radius~ : Double) -> Dubins raise @linear.Refused {
  abort("moonmcl: Dubins::new is not implemented yet")
}

///|
/// The shortest Dubins path between two `(x, y, heading)` poses.
pub fn Dubins::path(
  self : Dubins,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> DubinsPath raise @linear.Refused {
  abort("moonmcl: Dubins::path is not implemented yet")
}

///|
pub impl @contract.Space for Dubins with fn sample(self, rng) {
  abort("moonmcl: Dubins::sample is not implemented yet")
}

///|
pub impl @contract.Space for Dubins with fn distance(self, a, b) {
  abort("moonmcl: Dubins::distance is not implemented yet")
}

///|
pub impl @contract.Space for Dubins with fn interpolate(self, a, b, t) {
  abort("moonmcl: Dubins::interpolate is not implemented yet")
}

///|
pub extend Dubins with @contract.Space::{sample, distance, interpolate}

///|
/// A Reeds-Shepp car: forward and reverse at a fixed turning radius.
pub(all) struct ReedsShepp {
  turning_radius : Double
}

///|
/// A Reeds-Shepp car with the given radius.
pub fn ReedsShepp::new(
  turning_radius~ : Double,
) -> ReedsShepp raise @linear.Refused {
  abort("moonmcl: ReedsShepp::new is not implemented yet")
}

///|
/// The shortest Reeds-Shepp path between two `(x, y, heading)` poses, as a list
/// of gear, steering and length triples.
pub fn ReedsShepp::path(
  self : ReedsShepp,
  start~ : @linear.Vec,
  goal~ : @linear.Vec,
) -> Array[(Int32, Double, Double)] raise @linear.Refused {
  abort("moonmcl: ReedsShepp::path is not implemented yet")
}

///|
pub impl @contract.Space for ReedsShepp with fn sample(self, rng) {
  abort("moonmcl: ReedsShepp::sample is not implemented yet")
}

///|
pub impl @contract.Space for ReedsShepp with fn distance(self, a, b) {
  abort("moonmcl: ReedsShepp::distance is not implemented yet")
}

///|
pub impl @contract.Space for ReedsShepp with fn interpolate(self, a, b, t) {
  abort("moonmcl: ReedsShepp::interpolate is not implemented yet")
}

///|
pub extend ReedsShepp with @contract.Space::{sample, distance, interpolate}

///|
/// The dynamic window approach: sample the reachable velocities, roll each one
/// forward, keep the best.
pub(all) struct DynamicWindow {
  max_velocity : Double
  max_rotation : Double
  max_acceleration : Double
  max_rotation_acceleration : Double
  resolution_velocity : Double
  resolution_rotation : Double
  predict : Double
  clearance : Double
  heading_weight : Double
  clearance_weight : Double
  velocity_weight : Double
}

///|
/// The preset dynamic window.
pub let dynamic_window : DynamicWindow = {
  max_velocity: 1.0,
  max_rotation: 2.0,
  max_acceleration: 0.5,
  max_rotation_acceleration: 2.0,
  resolution_velocity: 0.05,
  resolution_rotation: 0.1,
  predict: 1.5,
  clearance: 0.5,
  heading_weight: 1.0,
  clearance_weight: 0.5,
  velocity_weight: 0.2,
}

///|
/// The best velocity pair for the current pose towards `goal`, avoiding
/// `obstacles`.
pub fn DynamicWindow::command(
  self : DynamicWindow,
  pose~ : @linear.Vec,
  velocity~ : @linear.Vec,
  goal~ : @linear.Vec,
  obstacles~ : ArrayView[@linear.Vec],
  dt~ : Double = 0.1,
) -> @linear.Vec raise @linear.Refused {
  abort("moonmcl: DynamicWindow::command is not implemented yet")
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
/// The velocity command for this step.
pub fn PotentialField::command(
  self : PotentialField,
  position~ : @linear.Vec,
  goal~ : @linear.Vec,
  obstacles~ : ArrayView[@linear.Vec],
) -> @linear.Vec {
  abort("moonmcl: PotentialField::command is not implemented yet")
}

///|
/// Which search a grid query runs.
#derive(Debug)
pub(all) enum GridSearch {
  Dijkstra
  AStar
  Jps
  ThetaStar
}

///|
/// The shortest path between two grid cells, as node indices, from the grid
/// package's own search.
pub fn grid_path(
  grid : @graph_grid.Grid,
  start~ : Int32,
  goal~ : Int32,
  search~ : GridSearch = AStar,
) -> Array[Int32] raise @linear.Refused {
  abort("moonmcl: grid_path is not implemented yet")
}

///|
/// A grid as a planning space, so a sampling planner can search a lattice.
pub(all) struct GridSpace {
  grid : @graph_grid.Grid
  resolution : Double
}

///|
/// A space whose states are grid cells scaled by `resolution`.
pub fn GridSpace::new(
  grid : @graph_grid.Grid,
  resolution~ : Double = 1.0,
) -> GridSpace raise @linear.Refused {
  abort("moonmcl: GridSpace::new is not implemented yet")
}

///|
pub impl @contract.Space for GridSpace with fn sample(self, rng) {
  abort("moonmcl: GridSpace::sample is not implemented yet")
}

///|
pub impl @contract.Space for GridSpace with fn distance(self, a, b) {
  abort("moonmcl: GridSpace::distance is not implemented yet")
}

///|
pub impl @contract.Space for GridSpace with fn interpolate(self, a, b, t) {
  abort("moonmcl: GridSpace::interpolate is not implemented yet")
}

///|
pub impl @contract.Validity for GridSpace with fn valid(self, x) {
  abort("moonmcl: GridSpace::valid is not implemented yet")
}

///|
pub extend GridSpace with @contract.Space::{sample, distance, interpolate}

///|
pub extend GridSpace with @contract.Validity::{valid, motion}

///|
pub extend DubinsWord with @debug.Debug::{to_repr}

///|
pub extend DubinsPath with @debug.Debug::{to_repr}

///|
pub extend PotentialField with @debug.Debug::{to_repr}

///|
pub extend GridSearch with @debug.Debug::{to_repr}
