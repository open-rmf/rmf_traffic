/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_TRAFFIC__AGV__PLANNER_HPP
#define RMF_TRAFFIC__AGV__PLANNER_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>

#include <rmf_utils/optional.hpp>

#include <atomic>

namespace rmf_traffic {
namespace agv {

//==============================================================================
// Forward declaration
class Plan;

//==============================================================================
class Planner
{
public:

  /// The Configuration class contains planning parameters that are immutable
  /// for each Planner instance.
  ///
  /// These parameters generally describe the capabilities or behaviors of the
  /// AGV that is being planned for, so they shouldn't need to change in between
  /// plans anyway.
  class Configuration
  {
  public:

    /// Constructor
    ///
    /// \param[in] vehicle_traits
    ///   The traits of the vehicle that is being planned for
    ///
    /// \param[in] graph
    ///   The graph which is being planned over
    ///
    /// \param[in] interpolation
    ///   The options for how the planner will perform trajectory interpolation
    Configuration(
      Graph graph,
      VehicleTraits traits,
      Interpolate::Options interpolation = Interpolate::Options());

    /// Set the graph to use for planning
    Configuration& graph(Graph graph);

    /// Get a mutable reference to the graph
    Graph& graph();

    /// Get a const reference to the graph
    const Graph& graph() const;

    /// Set the vehicle traits to use for planning
    Configuration& vehicle_traits(VehicleTraits traits);

    /// Get a mutable reference to the vehicle traits
    VehicleTraits& vehicle_traits();

    /// Get a const reference to the vehicle traits
    const VehicleTraits& vehicle_traits() const;

    /// Set the interpolation options for the planner
    Configuration& interpolation(Interpolate::Options interpolate);

    /// Get a mutable reference to the interpolation options
    Interpolate::Options& interpolation();

    /// Get a const reference to the interpolation options
    const Interpolate::Options& interpolation() const;

    /// Set the lane closures for the graph. The planner will not attempt to
    /// expand down any lanes that are closed.
    Configuration& lane_closures(LaneClosure closures);

    /// Get a mutable reference to the LaneClosure setting
    LaneClosure& lane_closures();

    /// Get a const reference to the LaneClosure setting
    const LaneClosure& lane_closures() const;

    /// How much the cost should increase per meter travelled. Besides this,
    /// cost is measured by the number of seconds spent travelling.
    Configuration& traversal_cost_per_meter(double value);

    /// Get the traversal cost.
    double traversal_cost_per_meter() const;

    // TODO(MXG): Add a field to specify whether multi-start planning problems
    // should choose the plan that takes the least amount of time (according to
    // plan duration) or the plan that finishes the earliest (according to the
    // wall clock).

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// The Options class contains planning parameters that can change between
  /// each planning attempt.
  class Options
  {
  public:

    static constexpr Duration DefaultMinHoldingTime = std::chrono::seconds(1);

    /// Constructor
    ///
    /// \param[in] validator
    ///   A validator to check the validity of the planner's branching options.
    ///
    /// \param[in] min_hold_time
    ///   The minimum amount of time that the planner should spend waiting at
    ///   holding points. Smaller values will make the plan more aggressive
    ///   about being time-optimal, but the plan may take longer to produce.
    ///   Larger values will add some latency to the execution of the plan as
    ///   the robot may wait at a holding point longer than necessary, but the
    ///   plan will usually be generated more quickly.
    ///
    /// \param[in] interrupt_flag
    ///   A pointer to a flag that should be used to interrupt the planner if it
    ///   has been running for too long. If the planner should run indefinitely,
    ///   then pass in a nullptr.
    ///
    /// \param[in] maximum_cost_estimate
    ///   A cap on how high the best possible solution's cost can be. If the
    ///   cost of the best possible solution ever exceeds this value, then the
    ///   planner will interrupt itself, no matter what the state of the
    ///   interrupt_flag is. Set this to nullopt to specify that there should
    ///   not be a cap.
    ///
    /// \param[in] saturation_limit
    ///   A cap on how many search nodes the planner is allowed to produce.
    Options(
      rmf_utils::clone_ptr<RouteValidator> validator,
      Duration min_hold_time = DefaultMinHoldingTime,
      std::shared_ptr<const std::atomic_bool> interrupt_flag = nullptr,
      std::optional<double> maximum_cost_estimate = std::nullopt,
      std::optional<std::size_t> saturation_limit = std::nullopt);

    /// Constructor
    ///
    /// \param[in] validator
    ///   A validator to check the validity of the planner's branching options.
    ///
    /// \param[in] validator
    ///   A validator to check the validity of the planner's branching options.
    ///
    /// \param[in] min_hold_time
    ///   The minimum amount of time that the planner should spend waiting at
    ///   holding points. Smaller values will make the plan more aggressive
    ///   about being time-optimal, but the plan may take longer to produce.
    ///   Larger values will add some latency to the execution of the plan as
    ///   the robot may wait at a holding point longer than necessary, but the
    ///   plan will usually be generated more quickly.
    ///
    /// \param[in] interrupter
    ///   A function that can determine whether the planning should be
    ///   interrupted. This is an alternative to using the interrupt_flag.
    ///
    /// \param[in] maximum_cost_estimate
    ///   A cap on how high the best possible solution's cost can be. If the
    ///   cost of the best possible solution ever exceeds this value, then the
    ///   planner will interrupt itself, no matter what the state of the
    ///   interrupt_flag is. Set this to nullopt to specify that there should
    ///   not be a cap.
    ///
    /// \param[in] saturation_limit
    ///   A cap on how many search nodes the planner is allowed to produce.
    Options(
      rmf_utils::clone_ptr<RouteValidator> validator,
      Duration min_hold_time,
      std::function<bool()> interrupter,
      std::optional<double> maximum_cost_estimate = std::nullopt,
      std::optional<std::size_t> saturation_limit = std::nullopt);

    /// Set the route validator
    Options& validator(rmf_utils::clone_ptr<RouteValidator> v);

    /// Get the route validator
    const rmf_utils::clone_ptr<RouteValidator>& validator() const;

    /// Set the minimum amount of time to spend waiting at holding points
    Options& minimum_holding_time(Duration holding_time);

    /// Get the minimum amount of time to spend waiting at holding points
    Duration minimum_holding_time() const;

    /// Set an interrupter callback that can indicate to the planner if it
    /// should stop trying to plan.
    ///
    /// \warning Using this function will replace anything that was given to
    /// interrupt_flag, and it will nullify the interrupt_flag() field.
    Options& interrupter(std::function<bool()> cb);

    /// Get the interrupter that will be used in these Options.
    const std::function<bool()>& interrupter() const;

    /// Set an interrupt flag to stop this planner if it has run for too long.
    ///
    /// \warning Using this function will replace anything that was given to
    /// interrupter.
    Options& interrupt_flag(std::shared_ptr<const std::atomic_bool> flag);

    /// Get the interrupt flag that will stop this planner if it has run for too
    /// long.
    const std::shared_ptr<const std::atomic_bool>& interrupt_flag() const;

    /// Set the maximum cost estimate that the planner should allow. If the cost
    /// estimate of the best possible plan that the planner could produce ever
    /// exceeds this value, the planner will pause itself (but this will not be
    /// considered an interruption).
    Options& maximum_cost_estimate(std::optional<double> value);

    /// Get the maximum cost estimate that the planner will allow.
    std::optional<double> maximum_cost_estimate() const;

    /// Set the saturation limit for the planner. If the planner produces more
    /// search nodes than this limit, then the planning will stop.
    Options& saturation_limit(std::optional<std::size_t> value);

    /// Get the saturation limit.
    std::optional<std::size_t> saturation_limit() const;

    /// Set the dependency window for generated plans. Any potential conflicts
    /// with the generated plan that happen within this window will be added as
    /// dependencies to the plan waypoints. If set to a nullopt, the plan will
    /// not have any dependencies.
    Options& dependency_window(std::optional<Duration> value);

    /// Dependency window for the planner.
    std::optional<Duration> dependency_window() const;

    /// Set the dependency resolution for generated plans. To check for
    /// dependencies, the planner will step the generated routes back in time by
    /// this value and check for conflicts. Detected conflicts get added to the
    /// list of dependencies. This backstepping happens until dependency_window
    /// is reached. If dependency_window is nullopt, this value will not be
    /// used.
    Options& dependency_resolution(Duration value);

    /// Get the dependency resolution for generated plans.
    Duration dependency_resolution() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Describe the starting conditions of a plan.
  class Start
  {
  public:

    /// Constructor
    ///
    /// \param[in] inital_time
    ///   The starting time of the plan.
    ///
    /// \param[in] initial_waypoint
    ///   The waypoint index that the plan will begin from.
    ///
    /// \param[in] initial_orientation
    ///   The orientation that the AGV will start with.
    ///
    /// \param[in] initial_location
    ///   Optional field to specify if the robot is not starting directly on the
    ///   initial_waypoint location. When planning from this initial_location to
    ///   the initial_waypoint the planner will assume it has an unconstrained
    ///   lane.
    ///
    /// \param[in] initial_lane
    ///   Optional field to specify if the robot is starting in a certain lane.
    ///   This will only be used if an initial_location is specified.
    Start(
      Time initial_time,
      std::size_t initial_waypoint,
      double initial_orientation,
      std::optional<Eigen::Vector2d> location = std::nullopt,
      std::optional<std::size_t> initial_lane = std::nullopt);

    /// Set the starting time of a plan
    Start& time(Time initial_time);

    /// Get the starting time
    Time time() const;

    /// Set the starting waypoint of a plan
    Start& waypoint(std::size_t initial_waypoint);

    /// Get the starting waypoint
    std::size_t waypoint() const;

    /// Set the starting orientation of a plan
    Start& orientation(double initial_orientation);

    /// Get the starting orientation
    double orientation() const;

    /// Get the starting location, if one was specified
    const std::optional<Eigen::Vector2d>& location() const;

    /// Set the starting location, or remove it by using std::nullopt
    Start& location(std::optional<Eigen::Vector2d> initial_location);

    /// Get the starting lane, if one was specified
    const std::optional<std::size_t>& lane() const;

    /// Set the starting lane, or remove it by using std::nullopt
    Start& lane(std::optional<std::size_t> initial_lane);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Describe the goal conditions of a plan.
  class Goal
  {
  public:

    // TODO(MXG): Consider using optional for the goal orientation

    // TODO(MXG): Consider supporting goals that have multiple acceptable goal
    // orientations.

    /// Constructor
    ///
    /// \note With this constructor, any final orientation will be accepted.
    ///
    /// \param[in] goal_waypoint
    ///   The waypoint that the AGV needs to reach.
    Goal(std::size_t goal_waypoint);

    /// Constructor
    ///
    /// \param[in] goal_waypoint
    ///   The waypoint that the AGV needs to reach.
    ///
    /// \param[in] goal_orientation
    ///   The orientation that the AGV needs to end with.
    Goal(std::size_t goal_waypoint, double goal_orientation);

    /// Constructor
    ///
    /// \param[in] goal_waypoint
    ///   The waypoint that the AGV needs to reach.
    ///
    /// \param[in] minimum_time
    ///   The AGV must be on the goal waypoint at or after this time for the
    ///   plan to be successful. This is useful if a robot needs to wait at a
    ///   location, but you want it to give way for other robots.
    ///
    /// \param[in] goal_orientation
    ///   An optional goal orientation that the AGV needs to end with.
    Goal(
      std::size_t goal_waypoint,
      std::optional<rmf_traffic::Time> minimum_time,
      std::optional<double> goal_orientation = std::nullopt);

    /// Set the goal waypoint.
    Goal& waypoint(std::size_t goal_waypoint);

    /// Get the goal waypoint.
    std::size_t waypoint() const;

    /// Set the goal orientation.
    Goal& orientation(double goal_orientation);

    /// Accept any orientation for the final goal.
    Goal& any_orientation();

    /// Get a reference to the goal orientation (or a nullptr if any orientation
    /// is acceptable).
    const double* orientation() const;

    /// Set the minimum time for the goal. Pass in a nullopt to remove the
    /// minimum time.
    Goal& minimum_time(std::optional<rmf_traffic::Time> value);

    /// Get the minimum time for the goal (or a nullopt is there is no minimum
    /// time).
    std::optional<rmf_traffic::Time> minimum_time() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  class Result;

  /// Constructor
  ///
  /// \param[in] config
  ///   This is the Configuration for the Planner. The Planner instance will
  ///   maintain a cache while it performs planning requests. This cache will
  ///   offer potential speed ups to subsequent planning requests, but the
  ///   correctness of the cache depends on the fields in the Configuration to
  ///   remain constant. Therefore you are not permitted to modify a Planner's
  ///   Configuration after the Planner is constructed. To change the planning
  ///   Configuration, you will need to create a new Planner instance with the
  ///   desired Configuration.
  ///
  /// \param[in] default_options
  ///   Unlike the Configuration, you are allowed to change a Planner's Options.
  ///   The parameter given here will be used as the default options, so you can
  ///   set them here and then forget about them. These options can be overriden
  ///   each time you request a plan.
  Planner(
    Configuration config,
    Options default_options);

  /// Get a const reference to the configuration for this Planner. Note that the
  /// configuration of a planner cannot be changed once it is set.
  ///
  /// \note The Planner maintains a cache that allows searches to become
  /// progressively faster. This cache depends on the fields in the Planner's
  /// configuration, so those fields cannot be changed without invalidating that
  /// cache. To plan using a different configuration, you should create a new
  /// Planner instance with the desired configuration.
  const Configuration& get_configuration() const;

  /// Change the default planning options.
  Planner& set_default_options(Options default_options);

  /// Get a mutable reference to the default planning options.
  Options& get_default_options();

  /// Get a const reference to the default planning options.
  const Options& get_default_options() const;

  using StartSet = std::vector<Start>;

  /// Produce a plan for the given starting conditions and goal. The default
  /// Options of this Planner instance will be used.
  ///
  /// \param[in] start
  ///   The starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  Result plan(const Start& start, Goal goal) const;

  /// Product a plan for the given start and goal conditions. Override the
  /// default options.
  ///
  /// \param[in] start
  ///   The starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  ///
  /// \param[in] options
  ///   The Options to use for this plan. This overrides the default Options of
  ///   the Planner instance.
  Result plan(
    const Start& start,
    Goal goal,
    Options options) const;


  /// Produces a plan for the given set of starting conditions and goal. The
  /// default Options of this Planner instance will be used.
  ///
  /// The planner will choose the start condition that allows for the shortest
  /// plan (not the one that finishes the soonest according to wall time).
  ///
  /// At least one start must be specified or else this is guaranteed to return
  /// a nullopt.
  ///
  /// \param[in] starts
  ///   The set of available starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  Result plan(const StartSet& starts, Goal goal) const;

  /// Produces a plan for the given set of starting conditions and goal.
  /// Override the default options.
  ///
  /// The planner will choose the start condition that allows for the shortest
  /// plan (not the one that finishes the soonest according to wall time).
  ///
  /// At least one start must be specified or else this is guaranteed to return
  /// a nullopt.
  ///
  /// \param[in] starts
  ///   The starting conditions
  ///
  /// \param[in] goal
  ///   The goal conditions
  ///
  /// \param[in] options
  ///   The options to use for this plan. This overrides the default Options of
  ///   the Planner instance.
  Result plan(
    const StartSet& starts,
    Goal goal,
    Options options) const;

  /// Set up a planning job, but do not start iterating.
  ///
  /// \sa plan(const Start&, Goal)
  Result setup(const Start& start, Goal goal) const;

  /// Set up a planning job, but do not start iterating.
  ///
  /// \sa plan(const Start&, Goal, Options)
  Result setup(
    const Start& start,
    Goal goal,
    Options options) const;

  /// Set up a planning job, but do not start iterating.
  ///
  /// \sa plan(const StartSet&, Goal)
  Result setup(const StartSet& starts, Goal goal) const;

  /// Set up a planning job, but do not start iterating.
  ///
  /// \sa plan(const StartSet&, Goal, Options)
  Result setup(
    const StartSet& starts,
    Goal goal,
    Options options) const;

  /// The quickest path is a simplified version of the Planner::Result class
  /// that can return faster but takes fewer factors into consideration. It
  /// does not account for translational acceleration/deceleration of the agent,
  /// nor does it consider rotational velocity or rotational acceleration. The
  /// traffic schedule is also not accounted for in this option.
  ///
  /// QuickestPath will not provide detailed timing information, instead only
  /// giving a simplified estimate of the cost and the graph indices for the
  /// quickest path.
  ///
  /// This quickest path does consider speed limits on lanes when estimating the
  /// quickest path. Besides the graph topology, lane closures, event duration
  /// estimates, and speed limits, no other planner Configuration or Option
  /// values are considered for the quickest path.
  ///
  /// This information is used internally as a heuristic for the full planner.
  /// The results of using this feature are cached and shared with the full
  /// planner, and vice-versa.
  class QuickestPath
  {
  public:
    /// The cost of following this path.
    double cost() const;

    /// The quickest path that was found.
    const std::vector<std::size_t>& path() const;

    class Implementation;
  private:
    QuickestPath();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Calculate the QuickestPath.
  ///
  /// \param[in] start
  ///   One or more start conditions to plan from. This function ignores time
  ///   and orientation information. Lane information is only used to know the
  ///   speed limit to use when moving from the start location to the initial
  ///   waypoint.
  ///
  ///   If the start set is left empty, the function will return a nullopt.
  ///
  /// \param[in] goal_vertex
  ///   The goal vertex to plan to.
  ///
  /// \return The quickest path from the start to the finish, or a nullopt if
  /// there is no path that connects the start to the finish.
  std::optional<QuickestPath> quickest_path(
      const StartSet& start,
      std::size_t goal_vertex) const;


  class CacheAudit;
  /// Get an audit of how much memory the planner's cache is using.
  CacheAudit cache_audit() const;

  /// Clear the cache for the differential drive heuristic. This will clear
  /// memory that the cache is occupying, but it will force the planner to
  /// rebuild the cache the next time it needs to generate a plan.
  ///
  /// This function can be used to bring down memory utilization in cases where
  /// the planner cache is ballooning. You can use the cache_audit function to
  /// identify when the cache is getting excessively large.
  ///
  /// Clearing the cache too frequently could harm the planner's performance.
  /// It is advisable to not clear the cache more than once per minute.
  void clear_differential_drive_cache() const;

  class Implementation;
  class Debug;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Planner::Result
{
public:

  /// True if a plan was found and this Result can be dereferenced to obtain a
  /// plan.
  bool success() const;

  /// True if there is no feasible path that connects the start to the goal.
  /// In this case, a plan will never be found.
  bool disconnected() const;

  /// Implicitly cast the result to a boolean. It will return true if a plan
  /// was found, otherwise it will return false.
  operator bool() const;

  /// If the Result was successful, drill into the plan.
  const Plan* operator->() const;

  /// If the Result was successful, get a reference to the plan.
  const Plan& operator*() const&;

  /// If the Result was successful, move the plan.
  Plan&& operator*() &&;

  /// If the Result was successful, get a reference to the plan.
  const Plan&& operator*() const&&;

  /// Replan to the same goal from a new start location using the same options
  /// as before.
  ///
  /// \param[in] new_start
  ///   The starting conditions that should be used for replanning.
  Result replan(const Start& new_start) const;

  /// Replan to the same goal from a new start location using a new set of
  /// options.
  ///
  /// \param[in] new_start
  ///   The starting conditions that should be used for replanning.
  ///
  /// \param[in] new_options
  ///   The options that should be used for replanning.
  Result replan(
    const Start& new_start,
    Options new_options) const;

  /// Replan to the same goal from a new set of start locations using the same
  /// options.
  ///
  /// \param[in] new_starts
  ///   The set of starting conditions that should be used for replanning.
  Result replan(const StartSet& new_starts) const;

  /// Replan to the same goal from a new set of start locations using a new set
  /// of options.
  ///
  /// \param[in] new_starts
  ///   The set of starting conditions that should be used for replanning.
  ///
  /// \param[in] new_options
  ///   The options that should be used for replanning.
  Result replan(
    const StartSet& new_starts,
    Options new_options) const;

  /// Set up a new planning job to the same goal, but do not start iterating.
  ///
  /// \sa replan(const Start&)
  Result setup(const Start& new_start) const;

  /// Set up a new planning job to the same goal, but do not start iterating.
  ///
  /// \sa replan(const Start&, Options)
  Result setup(
    const Start& new_start,
    Options new_options) const;

  /// Set up a new planning job to the same goal, but do not start iterating.
  ///
  /// \sa replan(const StartSet&)
  Result setup(const StartSet& new_starts) const;

  /// Set up a new planning job to the same goal, but do not start iterating.
  ///
  /// \sa replan(const StartSet&, Options)
  Result setup(
    const StartSet& new_starts,
    Options new_options) const;

  /// Resume planning if the planner was paused.
  ///
  /// \return true if a plan has been found, false otherwise.
  bool resume();

  /// Resume planning if the planner was paused.
  ///
  /// \param[in] interrupt_flag
  ///   A new interrupt flag to listen to while planning.
  ///
  /// \return true if a plan has been found, false otherwise.
  bool resume(std::shared_ptr<const std::atomic_bool> interrupt_flag);

  /// Get a mutable reference to the options that will be used by this planning
  /// task.
  Options& options();

  /// Get the options that will be used by this planning task.
  const Options& options() const;

  /// Change the options to be used by this planning task.
  Result& options(Options new_options);

  /// Get the best cost estimate of the current state of this planner result.
  /// This is the value of the lowest f(n)=g(n)+h(n) in the planner's queue.
  /// If the node queue of this planner result is empty, this will return a
  /// nullopt.
  std::optional<double> cost_estimate() const;

  /// Get the cost estimate that was initially computed for this plan. If no
  /// valid starts were provided, then this will return infinity.
  [[deprecated("Use ideal_cost() instead")]]
  double initial_cost_estimate() const;

  /// Get the cost that this plan would have if there is no traffic. If the plan
  /// is impossible (e.g. the starts are disconnected from the goal) this will
  /// return a nullopt.
  std::optional<double> ideal_cost() const;

  /// Get the start conditions that were given for this planning task.
  const std::vector<Start>& get_starts() const;

  /// Get the goal for this planning task.
  const Goal& get_goal() const;

  /// If this Plan is valid, this will return the Planner::Configuration that
  /// was used to produce it.
  ///
  /// If replan() is called, this Planner::Configuration will be used to produce
  /// the new Plan.
  const Configuration& get_configuration() const;

  /// This will return true if the planning failed because it was interrupted.
  /// Otherwise it will return false.
  bool interrupted() const;

  /// This will return true if the planner has reached its saturation limit.
  bool saturated() const;

  /// This is a list of schedule Participants who blocked the planning effort.
  /// Blockers do not necessarily prevent a solution from being found, but they
  /// do prevent the optimal solution from being available.
  std::vector<schedule::ParticipantId> blockers() const;



  class Implementation;
private:
  Result();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class Plan
{
public:

  using Start = Planner::Start;
  using StartSet = Planner::StartSet;
  using Goal = Planner::Goal;
  using Options = Planner::Options;
  using Configuration = Planner::Configuration;
  using Result = Planner::Result;

  struct Checkpoint
  {
    RouteId route_id;
    CheckpointId checkpoint_id;
  };
  using Checkpoints = std::vector<Checkpoint>;

  struct Progress
  {
    std::size_t graph_index;
    Checkpoints checkpoints;
    rmf_traffic::Time time;
  };

  /// A Waypoint within a Plan.
  ///
  /// This class helps to discretize a Plan based on the Waypoints belonging to
  /// the agv::Graph. Each Graph::Waypoint that the Plan stops or turns at will
  /// be accounted for by a Plan::Waypoint.
  ///
  /// To indicate the intended orientation, each of these Waypoints provides an
  /// Eigen::Vector3d where the third element is the orientation.
  ///
  /// The time that the position is meant to be arrived at is also given by the
  /// Waypoint.
  ///
  /// \note Users are not allowed to make their own Waypoint instances, because
  /// it is too easy to accidentally get inconsistencies in the position and
  /// graph_index fields. Plan::Waypoints can only be created by Plan instances
  /// and can only be retrieved using Plan::get_waypoints().
  class Waypoint
  {
  public:

    /// Get the position for this Waypoint
    const Eigen::Vector3d& position() const;

    /// Get the time for this Waypoint
    rmf_traffic::Time time() const;

    /// Get the graph index of this Waypoint
    std::optional<std::size_t> graph_index() const;

    /// Get the graph indices of the lanes that will be traversed on the way to
    /// this Waypoint. This will have multiple values if the robot is able to
    /// move straight through multiple lanes without stopping to reach this
    /// Waypoint. It will be empty if the robot does not need to traverse any
    /// lanes to reach this Waypoint (e.g. it is simply turning in place).
    const std::vector<std::size_t>& approach_lanes() const;

    /// Points on the graph that will be passed along the way to this waypoint.
    const std::vector<Progress>& progress_checkpoints() const;

    /// Points in the itinerary that have been reached when the robot arrives at
    /// this waypoint.
    const Checkpoints& arrival_checkpoints() const;

    [[deprecated("Use arrival_checkpoints().back().route_id instead")]]
    std::size_t itinerary_index() const;

    [[deprecated("Use arrival_checkpoints().back().checkpoint_id instead")]]
    std::size_t trajectory_index() const;

    /// An event that should occur when this waypoint is reached.
    const Graph::Lane::Event* event() const;

    /// The dependencies on other traffic participants that must be satisfied
    /// before leaving this waypoint.
    const Dependencies& dependencies() const;

    class Implementation;
  private:
    Waypoint();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// If this Plan is valid, this will return the trajectory of the successful
  /// plan. If the Start satisfies the Goal, then the itinerary will be empty.
  const std::vector<Route>& get_itinerary() const;

  /// If this plan is valid, this will return the waypoints of the successful
  /// plan.
  const std::vector<Waypoint>& get_waypoints() const;

  /// Get the start condition that was used for this plan.
  const Start& get_start() const;

  /// Get the final cost of this plan.
  double get_cost() const;

  // TODO(MXG): Create a feature that can diff two plans to produce the most
  // efficient schedule::Database::Change to get from the original plan to the
  // new plan.

  class Implementation;
private:
  Plan();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Number of elements in various caches within the planner.
class Planner::CacheAudit
{
public:

  std::size_t differential_drive_planner_cache_size() const;

  std::size_t shortest_path_cache_size() const;

  std::size_t euclidean_heuristic_cache_size() const;

  class Implementation;
private:
  CacheAudit();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// Produces a set of possible starting waypoints and lanes in order to start
/// planning. This method attempts to find the most suitable starting nodes
/// within the provided graph for merging, planning and execution of plans,
/// from the provided pose. If none of the waypoints in the graph fulfils the
/// requirements, an empty vector will be returned.
///
/// \param[in] graph
///   Graph which the starting waypoints and lanes will be derived from.
///
/// \param[in] pose
///   Current pose in terms of 2D coordinates, x and y, being the first and
///   second element respectively, while the third element being the yaw.
///
/// \param[in] start_time
///   The starting time that will be attributed to all the generated starts
///   to compute a new plan. In some occasions, users will want to add small
///   delays to the current time, in order to account for computation time or
///   network delays.
///
/// \param[in] max_merge_waypoint_distance
///   The maximum distance allowed to automatically merge onto a waypoint in
///   the graph. Default value as 0.1 meters.
///
/// \param[in] max_merge_lane_distance
///   The maximum distance allowed to automatically merge onto a lane, i.e.
///   adding the lane's entry and exit waypoints as potential starts. Default
///   value as 1.0 meters.
///
/// \param[in] min_lane_length
///   The minimum length of a lane in the provided graph to be considered valid,
///   any lanes shorter than this value will not be evaluated. Default value as
///   1e-8 meters.
std::vector<Plan::Start> compute_plan_starts(
  const rmf_traffic::agv::Graph& graph,
  const std::string& map_name,
  const Eigen::Vector3d pose,
  const rmf_traffic::Time start_time,
  const double max_merge_waypoint_distance = 0.1,
  const double max_merge_lane_distance = 1.0,
  const double min_lane_length = 1e-8);

} // namespace agv
} // namespace rmf_traffic

namespace std {

//==============================================================================
ostream& operator<<(
  ostream& os,
  const rmf_traffic::agv::Planner::CacheAudit& audit);

} // namespace std

#endif // RMF_TRAFFIC__AGV__PLANNER_HPP
