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

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>

#include "internal_Planner.hpp"
#include "internal_planning.hpp"

namespace rmf_traffic {
namespace agv {

//==============================================================================
// This line tells the linker to take care of defining the value of this field
// inside of this translation unit.
const Duration Planner::Options::DefaultMinHoldingTime;

//==============================================================================
class Planner::Configuration::Implementation
{
public:

  Graph graph;
  VehicleTraits traits;
  Interpolate::Options interpolation;
  LaneClosure lane_closures;
  double traversal_cost_per_meter = 5.0;

};

//==============================================================================
Planner::Configuration::Configuration(
  Graph graph,
  VehicleTraits traits,
  Interpolate::Options interpolation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(graph),
        std::move(traits),
        std::move(interpolation),
        LaneClosure()
      }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Configuration::graph(Graph graph) -> Configuration&
{
  _pimpl->graph = std::move(graph);
  return *this;
}

//==============================================================================
Graph& Planner::Configuration::graph()
{
  return _pimpl->graph;
}

//==============================================================================
const Graph& Planner::Configuration::graph() const
{
  return _pimpl->graph;
}

//==============================================================================
auto Planner::Configuration::vehicle_traits(VehicleTraits traits)
-> Configuration&
{
  _pimpl->traits = std::move(traits);
  return *this;
}

//==============================================================================
VehicleTraits& Planner::Configuration::vehicle_traits()
{
  return _pimpl->traits;
}

//==============================================================================
const VehicleTraits& Planner::Configuration::vehicle_traits() const
{
  return _pimpl->traits;
}

//==============================================================================
auto Planner::Configuration::interpolation(Interpolate::Options interpolate)
-> Configuration&
{
  _pimpl->interpolation = std::move(interpolate);
  return *this;
}

//==============================================================================
Interpolate::Options& Planner::Configuration::interpolation()
{
  return _pimpl->interpolation;
}

//==============================================================================
const Interpolate::Options& Planner::Configuration::interpolation() const
{
  return _pimpl->interpolation;
}

//==============================================================================
auto Planner::Configuration::lane_closures(LaneClosure closures)
-> Configuration&
{
  _pimpl->lane_closures = std::move(closures);
  return *this;
}

//==============================================================================
LaneClosure& Planner::Configuration::lane_closures()
{
  return _pimpl->lane_closures;
}

//==============================================================================
const LaneClosure& Planner::Configuration::lane_closures() const
{
  return _pimpl->lane_closures;
}

//==============================================================================
auto Planner::Configuration::traversal_cost_per_meter(
  double per_meter) -> Configuration&
{
  _pimpl->traversal_cost_per_meter = per_meter;
  return *this;
}

//==============================================================================
double Planner::Configuration::traversal_cost_per_meter() const
{
  return _pimpl->traversal_cost_per_meter;
}

//==============================================================================
class Planner::Options::Implementation
{
public:

  rmf_utils::clone_ptr<RouteValidator> validator;
  Duration min_hold_time;
  std::optional<double> maximum_cost_estimate;
  std::optional<std::size_t> saturation_limit;

  std::function<bool()> interrupter = nullptr;
  std::shared_ptr<const std::atomic_bool> interrupt_flag = nullptr;

  std::optional<Duration> dependency_window = std::chrono::seconds(30);
  Duration dependency_resolution = std::chrono::milliseconds(1000);

};

//==============================================================================
Planner::Options::Options(
  rmf_utils::clone_ptr<RouteValidator> validator,
  const Duration min_hold_time,
  std::shared_ptr<const std::atomic_bool> interrupt_flag,
  std::optional<double> maximum_cost_estimate,
  std::optional<std::size_t> saturation_limit)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(validator),
        min_hold_time,
        maximum_cost_estimate,
        saturation_limit
      }))
{
  this->interrupt_flag(std::move(interrupt_flag));
}

//==============================================================================
Planner::Options::Options(
  rmf_utils::clone_ptr<RouteValidator> validator,
  const Duration min_hold_time,
  std::function<bool()> interrupter,
  std::optional<double> maximum_cost_estimate,
  std::optional<std::size_t> saturation_limit)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(validator),
        min_hold_time,
        maximum_cost_estimate,
        saturation_limit,
        std::move(interrupter)
      }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Options::validator(rmf_utils::clone_ptr<RouteValidator> v)
-> Options&
{
  _pimpl->validator = std::move(v);
  return *this;
}

//==============================================================================
const rmf_utils::clone_ptr<RouteValidator>& Planner::Options::validator() const
{
  return _pimpl->validator;
}

//==============================================================================
auto Planner::Options::minimum_holding_time(const Duration holding_time)
-> Options&
{
  _pimpl->min_hold_time = holding_time;
  return *this;
}

//==============================================================================
Duration Planner::Options::minimum_holding_time() const
{
  return _pimpl->min_hold_time;
}

//==============================================================================
auto Planner::Options::interrupter(std::function<bool()> cb) -> Options&
{
  _pimpl->interrupt_flag = nullptr;
  _pimpl->interrupter = std::move(cb);
  return *this;
}

//==============================================================================
const std::function<bool()>& Planner::Options::interrupter() const
{
  return _pimpl->interrupter;
}

//==============================================================================
auto Planner::Options::interrupt_flag(
  std::shared_ptr<const std::atomic_bool> flag) -> Options&
{
  if (flag)
  {
    _pimpl->interrupt_flag = flag;
    _pimpl->interrupter = [flag = std::move(flag)]() -> bool
      { return flag->load(std::memory_order::memory_order_relaxed); };
  }
  else
  {
    _pimpl->interrupt_flag = nullptr;
    _pimpl->interrupter = nullptr;
  }

  return *this;
}

//==============================================================================
const std::shared_ptr<const std::atomic_bool>&
Planner::Options::interrupt_flag() const
{
  return _pimpl->interrupt_flag;
}

//==============================================================================
auto Planner::Options::maximum_cost_estimate(std::optional<double> value)
-> Options&
{
  _pimpl->maximum_cost_estimate = value;
  return *this;
}

//==============================================================================
std::optional<double> Planner::Options::maximum_cost_estimate() const
{
  return _pimpl->maximum_cost_estimate;
}

//==============================================================================
auto Planner::Options::saturation_limit(std::optional<std::size_t> value)
-> Options&
{
  _pimpl->saturation_limit = value;
  return *this;
}

//==============================================================================
std::optional<std::size_t> Planner::Options::saturation_limit() const
{
  return _pimpl->saturation_limit;
}

//==============================================================================
auto Planner::Options::dependency_window(std::optional<Duration> value)
-> Options&
{
  _pimpl->dependency_window = value;
  return *this;
}

//==============================================================================
std::optional<Duration> Planner::Options::dependency_window() const
{
  return _pimpl->dependency_window;
}

//==============================================================================
auto Planner::Options::dependency_resolution(Duration value) -> Options&
{
  _pimpl->dependency_resolution = value;
  return *this;
}

//==============================================================================
Duration Planner::Options::dependency_resolution() const
{
  return _pimpl->dependency_resolution;
}

//==============================================================================
class Planner::Start::Implementation
{
public:

  Time time;
  std::size_t waypoint;
  double orientation;
  std::optional<Eigen::Vector2d> location;
  std::optional<std::size_t> lane;

};

//==============================================================================
Planner::Start::Start(
  const Time initial_time,
  const std::size_t initial_waypoint,
  const double initial_orientation,
  std::optional<Eigen::Vector2d> initial_location,
  std::optional<std::size_t> initial_lane)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        initial_time,
        initial_waypoint,
        initial_orientation,
        std::move(initial_location),
        std::move(initial_lane)
      }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Start::time(const Time initial_time) -> Start&
{
  _pimpl->time = initial_time;
  return *this;
}

//==============================================================================
Time Planner::Start::time() const
{
  return _pimpl->time;
}

//==============================================================================
auto Planner::Start::waypoint(const std::size_t initial_waypoint) -> Start&
{
  _pimpl->waypoint = initial_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Start::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Start::orientation(const double initial_orientation) -> Start&
{
  _pimpl->orientation = initial_orientation;
  return *this;
}

//==============================================================================
double Planner::Start::orientation() const
{
  return _pimpl->orientation;
}

//==============================================================================
const std::optional<Eigen::Vector2d>& Planner::Start::location() const
{
  return _pimpl->location;
}

//==============================================================================
auto Planner::Start::location(
  std::optional<Eigen::Vector2d> initial_location) -> Start&
{
  _pimpl->location = std::move(initial_location);
  return *this;
}

//==============================================================================
const std::optional<std::size_t>& Planner::Start::lane() const
{
  return _pimpl->lane;
}

//==============================================================================
auto Planner::Start::lane(
  std::optional<std::size_t> initial_lane) -> Start&
{
  _pimpl->lane = initial_lane;
  return *this;
}

//==============================================================================
class Planner::Goal::Implementation
{
public:

  std::size_t waypoint;
  std::optional<double> orientation;
  std::optional<rmf_traffic::Time> minimum_time;

};

//==============================================================================
Planner::Goal::Goal(const std::size_t waypoint)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        waypoint,
        std::nullopt,
        std::nullopt
      }))
{
  // Do nothing
}

//==============================================================================
Planner::Goal::Goal(
  const std::size_t waypoint,
  const double goal_orientation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        waypoint,
        goal_orientation,
        std::nullopt
      }))
{
  // Do nothing
}

//==============================================================================
Planner::Goal::Goal(
  const std::size_t goal_waypoint,
  const std::optional<rmf_traffic::Time> minimum_time,
  const std::optional<double> goal_orientation)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        goal_waypoint,
        goal_orientation,
        minimum_time
      }))
{
  // Do nothing
}

//==============================================================================
auto Planner::Goal::waypoint(const std::size_t goal_waypoint) -> Goal&
{
  _pimpl->waypoint = goal_waypoint;
  return *this;
}

//==============================================================================
std::size_t Planner::Goal::waypoint() const
{
  return _pimpl->waypoint;
}

//==============================================================================
auto Planner::Goal::orientation(const double goal_orientation) -> Goal&
{
  _pimpl->orientation = goal_orientation;
  return *this;
}

//==============================================================================
auto Planner::Goal::any_orientation() -> Goal&
{
  _pimpl->orientation = rmf_utils::nullopt;
  return *this;
}

//==============================================================================
const double* Planner::Goal::orientation() const
{
  if (_pimpl->orientation)
    return &(*_pimpl->orientation);

  return nullptr;
}

//==============================================================================
auto Planner::Goal::minimum_time(std::optional<rmf_traffic::Time> value)
-> Goal&
{
  _pimpl->minimum_time = value;
  return *this;
}

//==============================================================================
std::optional<rmf_traffic::Time> Planner::Goal::minimum_time() const
{
  return _pimpl->minimum_time;
}

//==============================================================================
class Planner::Implementation
{
public:

  planning::InterfacePtr interface;

  Options default_options;

  Configuration configuration;

};

//==============================================================================
class Plan::Implementation
{
public:

  planning::PlanData plan;

  static std::optional<Plan> make(std::optional<planning::PlanData> result)
  {
    if (!result)
      return std::nullopt;

    Plan plan;
    plan._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{*std::move(result)});

    return plan;
  }

};

//==============================================================================
Planner::Planner(
  Configuration config,
  Options default_options)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        planning::make_planner_interface(config),
        std::move(default_options),
        config
      }))
{
  // Do nothing
}

//==============================================================================
Planner::Result Planner::Result::Implementation::generate(
  planning::InterfacePtr interface,
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options)
{
  // TODO(MXG): Throw an exception if any of the starts or the goal has an
  // invalid waypoint index.
  auto state = interface->initiate(
    starts, std::move(goal), std::move(options));

  auto plan = Plan::Implementation::make(interface->plan(state));

  Planner::Result result;
  result._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      std::move(interface),
      std::move(state),
      std::move(plan)
    });

  return result;
}

//==============================================================================
Planner::Result Planner::Result::Implementation::setup(
  planning::InterfacePtr interface,
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options)
{
  auto state = interface->initiate(
    starts, std::move(goal), std::move(options));

  Planner::Result result;
  result._pimpl = rmf_utils::make_impl<Implementation>(
    Implementation{
      std::move(interface),
      std::move(state),
      rmf_utils::nullopt
    });

  return result;
}

//==============================================================================
auto Planner::Result::Implementation::get(const Result& r)
-> const Implementation&
{
  return *r._pimpl;
}

//==============================================================================
auto Planner::get_configuration() const -> const Configuration&
{
  return _pimpl->configuration;
}

//==============================================================================
Planner& Planner::set_default_options(Options default_options)
{
  _pimpl->default_options = std::move(default_options);
  return *this;
}

//==============================================================================
auto Planner::get_default_options() -> Options&
{
  return _pimpl->default_options;
}

//==============================================================================
auto Planner::get_default_options() const -> const Options&
{
  return _pimpl->default_options;
}

//==============================================================================
Planner::Result Planner::plan(const Start& start, Goal goal) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    {start},
    std::move(goal),
    _pimpl->default_options);
}

//==============================================================================
Planner::Result Planner::plan(
  const Start& start,
  Goal goal,
  Options options) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    {start},
    std::move(goal),
    std::move(options));
}

//==============================================================================
Planner::Result Planner::plan(const StartSet& starts, Goal goal) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    starts,
    std::move(goal),
    _pimpl->default_options);
}

//==============================================================================
Planner::Result Planner::plan(
  const StartSet& starts,
  Goal goal,
  Options options) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    starts,
    std::move(goal),
    std::move(options));
}

//==============================================================================
Planner::Result Planner::setup(const Start& start, Goal goal) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    {start},
    std::move(goal),
    _pimpl->default_options);
}

//==============================================================================
Planner::Result Planner::setup(
  const Start& start,
  Goal goal,
  Options options) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    {start},
    std::move(goal),
    std::move(options));
}

//==============================================================================
Planner::Result Planner::setup(const StartSet& start, Goal goal) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    start,
    std::move(goal),
    _pimpl->default_options);
}

//==============================================================================
Planner::Result Planner::setup(
  const StartSet& start,
  Goal goal,
  Options options) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    start,
    std::move(goal),
    std::move(options));
}

//==============================================================================
bool Planner::Result::success() const
{
  return _pimpl->plan.has_value();
}

//==============================================================================
bool Planner::Result::disconnected() const
{
  return _pimpl->state.issues.disconnected;
}

//==============================================================================
Planner::Result::operator bool() const
{
  return _pimpl->plan.has_value();
}

//==============================================================================
const Plan* Planner::Result::operator->() const
{
  return &(*_pimpl->plan);
}

//==============================================================================
const Plan& Planner::Result::operator*() const&
{
  return *_pimpl->plan;
}

//==============================================================================
Plan&& Planner::Result::operator*() &&
{
  return std::move(*std::move(_pimpl->plan));
}

//==============================================================================
const Plan&& Planner::Result::operator*() const&&
{
  return std::move(*_pimpl->plan);
}

//==============================================================================
Planner::Result Planner::Result::replan(const Start& new_start) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    {new_start},
    _pimpl->state.conditions.goal,
    _pimpl->state.conditions.options);
}

//==============================================================================
Planner::Result Planner::Result::replan(
  const Planner::Start& new_start,
  Planner::Options new_options) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    {new_start},
    _pimpl->state.conditions.goal,
    std::move(new_options));
}

//==============================================================================
Planner::Result Planner::Result::replan(const StartSet& new_starts) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    new_starts,
    _pimpl->state.conditions.goal,
    _pimpl->state.conditions.options);
}

//==============================================================================
Planner::Result Planner::Result::replan(
  const StartSet& new_starts,
  Options new_options) const
{
  return Result::Implementation::generate(
    _pimpl->interface,
    new_starts,
    _pimpl->state.conditions.goal,
    std::move(new_options));
}

//==============================================================================
Planner::Result Planner::Result::setup(const Start& new_start) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    {new_start},
    _pimpl->state.conditions.goal,
    _pimpl->state.conditions.options);
}

//==============================================================================
Planner::Result Planner::Result::setup(
  const Start& new_start,
  Options new_options) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    {new_start},
    _pimpl->state.conditions.goal,
    std::move(new_options));
}

//==============================================================================
Planner::Result Planner::Result::setup(const StartSet& new_starts) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    new_starts,
    _pimpl->state.conditions.goal,
    _pimpl->state.conditions.options);
}

//==============================================================================
Planner::Result Planner::Result::setup(
  const StartSet& new_starts,
  Options new_options) const
{
  return Result::Implementation::setup(
    _pimpl->interface,
    new_starts,
    _pimpl->state.conditions.goal,
    std::move(new_options));
}

//==============================================================================
bool Planner::Result::resume()
{
  if (_pimpl->plan)
    return true;

  _pimpl->plan = Plan::Implementation::make(
    _pimpl->interface->plan(_pimpl->state));

  return _pimpl->plan.has_value();
}

//==============================================================================
bool Planner::Result::resume(
  std::shared_ptr<const std::atomic_bool> interrupt_flag)
{
  _pimpl->state.conditions.options.interrupt_flag(std::move(interrupt_flag));
  return resume();
}

//==============================================================================
Planner::Options& Planner::Result::options()
{
  return _pimpl->state.conditions.options;
}

//==============================================================================
const Planner::Options& Planner::Result::options() const
{
  return _pimpl->state.conditions.options;
}

//==============================================================================
Planner::Result& Planner::Result::options(Options new_options)
{
  _pimpl->state.conditions.options = std::move(new_options);
  return *this;
}

//==============================================================================
std::optional<double> Planner::Result::cost_estimate() const
{
  return _pimpl->state.internal->cost_estimate();
}

//==============================================================================
double Planner::Result::initial_cost_estimate() const
{
  return _pimpl->state.ideal_cost.value_or(
    std::numeric_limits<double>::infinity());
}

//==============================================================================
std::optional<double> Planner::Result::ideal_cost() const
{
  return _pimpl->state.ideal_cost;
}

//==============================================================================
const std::vector<Planner::Start>& Planner::Result::get_starts() const
{
  return _pimpl->state.conditions.starts;
}

//==============================================================================
const Planner::Goal& Planner::Result::get_goal() const
{
  return _pimpl->state.conditions.goal;
}

//==============================================================================
const Planner::Configuration& Planner::Result::get_configuration() const
{
  return _pimpl->interface->get_configuration();
}

//==============================================================================
bool Planner::Result::interrupted() const
{
  return _pimpl->state.issues.interrupted;
}

//==============================================================================
bool Planner::Result::saturated() const
{
  const auto saturation_limit =
    _pimpl->state.conditions.options.saturation_limit();

  if (!saturation_limit)
    return false;

  const std::size_t saturation =
    _pimpl->state.internal->queue_size() + _pimpl->state.popped_count;

  return *saturation_limit < saturation;
}

//==============================================================================
std::vector<schedule::ParticipantId> Planner::Result::blockers() const
{
  std::vector<schedule::ParticipantId> blockers;
  blockers.reserve(_pimpl->state.issues.blocked_nodes.size());
  for (const auto& b : _pimpl->state.issues.blocked_nodes)
    blockers.push_back(b.first);

  return blockers;
}

//==============================================================================
Planner::Result::Result()
{
  // Do nothing
}

//==============================================================================
double Planner::QuickestPath::cost() const
{
  return _pimpl->solution->cost + _pimpl->cost_offset;
}

//==============================================================================
const std::vector<std::size_t>& Planner::QuickestPath::path() const
{
  return _pimpl->solution->path;
}

//==============================================================================
Planner::QuickestPath::QuickestPath()
{
  // Do nothing
}

//==============================================================================
void Planner::QuickestPath::Implementation::choose_better(
  std::optional<Implementation>& left,
  const Implementation& right)
{
  if (!left.has_value())
  {
    left = right;
    return;
  }

  const auto c_left = left->cost_offset + left->solution->cost;
  const auto c_right = right.cost_offset + right.solution->cost;

  if (c_right < c_left)
    left = right;
}

//==============================================================================
std::optional<Planner::QuickestPath>
Planner::QuickestPath::Implementation::promote(
  std::optional<Implementation> value)
{
  if (!value.has_value())
    return std::nullopt;

  QuickestPath output;
  output._pimpl = rmf_utils::make_impl<Implementation>(*std::move(value));

  return output;
}

//==============================================================================
auto Planner::quickest_path(
  const StartSet& start,
  const std::size_t goal) const
-> std::optional<QuickestPath>
{
  return _pimpl->interface->quickest_path(start, goal);
}

//==============================================================================
auto Planner::cache_audit() const -> CacheAudit
{
  return _pimpl->interface->cache_audit();
}

//==============================================================================
void Planner::clear_differential_drive_cache() const
{
  _pimpl->interface->clear_cache();
}

//==============================================================================
const Eigen::Vector3d& Plan::Waypoint::position() const
{
  return _pimpl->position;
}

//==============================================================================
rmf_traffic::Time Plan::Waypoint::time() const
{
  return _pimpl->time;
}

//==============================================================================
std::optional<std::size_t> Plan::Waypoint::graph_index() const
{
  return _pimpl->graph_index;
}

//==============================================================================
const std::vector<std::size_t>& Plan::Waypoint::approach_lanes() const
{
  return _pimpl->approach_lanes;
}

//==============================================================================
auto Plan::Waypoint::progress_checkpoints() const
-> const std::vector<Progress>&
{
  return _pimpl->progress;
}

//==============================================================================
auto Plan::Waypoint::arrival_checkpoints() const -> const Checkpoints&
{
  return _pimpl->arrival;
}

//==============================================================================
std::size_t Plan::Waypoint::itinerary_index() const
{
  return _pimpl->arrival.back().route_id;
}

//==============================================================================
std::size_t Plan::Waypoint::trajectory_index() const
{
  return _pimpl->arrival.back().checkpoint_id;
}

//==============================================================================
const Graph::Lane::Event* Plan::Waypoint::event() const
{
  return _pimpl->event.get();
}

//==============================================================================
const Dependencies& Plan::Waypoint::dependencies() const
{
  return _pimpl->dependencies;
}

//==============================================================================
Plan::Waypoint::Waypoint()
{
  // Do nothing
}

//==============================================================================
const std::vector<Route>& Plan::get_itinerary() const
{
  return _pimpl->plan.routes;
}

//==============================================================================
const std::vector<Plan::Waypoint>& Plan::get_waypoints() const
{
  return _pimpl->plan.waypoints;
}

//==============================================================================
const Plan::Start& Plan::get_start() const
{
  return _pimpl->plan.start;
}

//==============================================================================
double Plan::get_cost() const
{
  return _pimpl->plan.cost;
}

//==============================================================================
Plan::Plan()
{
  // Do nothing
}

//==============================================================================
Planner::CacheAudit::CacheAudit()
  : _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
std::size_t Planner::CacheAudit::differential_drive_planner_cache_size() const
{
  return _pimpl->differential_drive_planner_cache_size;
}

//==============================================================================
std::size_t Planner::CacheAudit::shortest_path_cache_size() const
{
  return _pimpl->shortest_path_cache_size;
}

//==============================================================================
std::size_t Planner::CacheAudit::euclidean_heuristic_cache_size() const
{
  return _pimpl->euclidean_heuristic_cache_size;
}

//==============================================================================
std::vector<Plan::Start> compute_plan_starts(
  const rmf_traffic::agv::Graph& graph,
  const std::string& map_name,
  const Eigen::Vector3d pose,
  const rmf_traffic::Time start_time,
  const double max_merge_waypoint_distance,
  const double max_merge_lane_distance,
  const double min_lane_length)
{
  const Eigen::Vector2d p_location = {pose[0], pose[1]};
  const double start_yaw = pose[2];

  // If there are waypoints which are very close, take that as the only Start
  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    const auto& wp = graph.get_waypoint(i);
    if (wp.get_map_name() != map_name)
      continue;

    const Eigen::Vector2d wp_location = wp.get_location();
    const auto merge_radius =
      wp.merge_radius().value_or(max_merge_waypoint_distance);

    if ( (p_location - wp_location).norm() < merge_radius)
    {
      return {Plan::Start(start_time, wp.index(), start_yaw)};
    }
  }

  // Iterate through the lanes and return the set of possible waypoints, i.e.
  // entries and exits of nearby lanes.
  std::vector<Plan::Start> starts;
  std::unordered_set<std::size_t> raw_starts;

  for (std::size_t i = 0; i < graph.num_lanes(); ++i)
  {
    const auto& lane = graph.get_lane(i);
    const auto& wp0 = graph.get_waypoint(lane.entry().waypoint_index());
    const auto& wp1 = graph.get_waypoint(lane.exit().waypoint_index());
    if (wp0.get_map_name() != map_name && wp1.get_map_name() != map_name)
      continue;

    const std::optional<double> wp0_merge_radius = wp0.merge_radius();
    const std::optional<double> wp1_merge_radius = wp1.merge_radius();

    const Eigen::Vector2d p0 = wp0.get_location();
    const Eigen::Vector2d p1 = wp1.get_location();

    const double lane_length = (p1 - p0).norm();

    // This "lane" is either two points stacked very close or is moving
    // vertically through a lift.
    const double merge_dist = std::max(
      wp0_merge_radius.value_or(max_merge_lane_distance),
      wp1_merge_radius.value_or(max_merge_lane_distance));

    if (lane_length < min_lane_length)
    {
      const double dp0 = (p_location - p0).norm();
      const double dp1 = (p_location - p1).norm();
      if (dp0 < merge_dist || dp1 < merge_dist)
      {
        starts.emplace_back(
          Plan::Start(
            start_time, lane.exit().waypoint_index(),
            start_yaw, p_location, i));
      }

      continue;
    }

    const Eigen::Vector2d pn = (p1 - p0) / lane_length;
    const Eigen::Vector2d p_l = p_location - p0;
    const double p_l_projection = p_l.dot(pn);

    // If it's negative then its closest point on the lane is the entry point
    if (p_l_projection < 0.0)
    {
      const double dist_to_entry = p_l.norm();
      const std::size_t entry_waypoint_index = lane.entry().waypoint_index();

      const double merge_dist =
        wp0_merge_radius.value_or(max_merge_lane_distance);
      if (dist_to_entry < merge_dist)
      {
        if (!raw_starts.insert(entry_waypoint_index).second)
          continue;

        starts.emplace_back(
          Plan::Start(
            start_time, entry_waypoint_index, start_yaw, p_location));
      }
    }
    // If it's larger than the lane length, then its closest point on the lane
    // is the exit point.
    else if (lane_length < p_l_projection)
    {
      const double dist_to_exit = (p_location - p1).norm();
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();

      const double merge_dist =
        wp1_merge_radius.value_or(max_merge_lane_distance);
      if (dist_to_exit < merge_dist)
      {
        if (!raw_starts.insert(exit_waypoint_index).second)
          continue;

        starts.emplace_back(
          Plan::Start(
            start_time, exit_waypoint_index, start_yaw, p_location));
      }
    }
    // If its between the entry and the exit waypoints, then we should
    // compute it's distance away from the lane line.
    else
    {
      const double lane_dist = (p_l - p_l_projection*pn).norm();
      const std::size_t exit_waypoint_index = lane.exit().waypoint_index();
      double merge_dist = max_merge_lane_distance;
      if (wp0_merge_radius.has_value() && wp1_merge_radius.has_value())
        merge_dist = std::max(*wp0_merge_radius, *wp1_merge_radius);

      if (lane_dist < merge_dist)
      {
        starts.emplace_back(
          Plan::Start(
            start_time, exit_waypoint_index, start_yaw, p_location, i));
      }
    }
  }

  return starts;
}

//==============================================================================
class Planner::Debug::Implementation
{
public:

  planning::InterfacePtr interface;

};

//==============================================================================
class Planner::Debug::Progress::Implementation
{
public:
  planning::InterfacePtr interface;
  std::unique_ptr<planning::Interface::Debugger> debugger;

  Implementation(
    planning::InterfacePtr interface_,
    const std::vector<Start>& starts,
    Goal goal,
    Options options)
  : interface(std::move(interface_)),
    debugger(interface->debug_begin(
        starts, std::move(goal), std::move(options)))
  {
    // Do nothing
  }

  static Progress make(
    planning::InterfacePtr interface,
    const std::vector<Start>& starts,
    Goal goal,
    Options options)
  {
    Progress progress;
    progress._pimpl = rmf_utils::make_unique_impl<Implementation>(
      Implementation{interface, starts, std::move(goal), std::move(options)});

    return progress;
  }
};

//==============================================================================
std::optional<Plan> Planner::Debug::Progress::step()
{
  auto result = _pimpl->interface->debug_step(*_pimpl->debugger);

  if (!result)
    return rmf_utils::nullopt;

  return Plan::Implementation::make(std::move(*result));
}

//==============================================================================
auto Planner::Debug::Progress::queue() const -> const Node::SearchQueue&
{
  return _pimpl->debugger->queue();
}

//==============================================================================
auto Planner::Debug::Progress::expanded_nodes() const -> const Node::Vector&
{
  return _pimpl->debugger->expanded_nodes();
}

//==============================================================================
auto Planner::Debug::Progress::terminal_nodes() const -> const Node::Vector&
{
  return _pimpl->debugger->terminal_nodes();
}

//==============================================================================
Planner::Debug::Progress::Progress()
{
  // Do nothing
}

//==============================================================================
Planner::Debug::Debug(const Planner& planner)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{planner._pimpl->interface}))
{
  // Do nothing
}

//==============================================================================
auto Planner::Debug::begin(
  const std::vector<Start>& starts,
  Goal goal,
  Options options) const -> Progress
{
  return Progress::Implementation::make(
    _pimpl->interface,
    starts,
    std::move(goal),
    std::move(options));
}

//==============================================================================
std::size_t Planner::Debug::queue_size(const Planner::Result& result)
{
  return Planner::Result::Implementation::get(result)
    .state.internal->queue_size();
}

//==============================================================================
std::size_t Planner::Debug::expansion_count(const Planner::Result& result)
{
  return Planner::Result::Implementation::get(result)
    .state.internal->expansion_count();
}

//==============================================================================
std::size_t Planner::Debug::node_count(const Planner::Result& result)
{
  return queue_size(result) + expansion_count(result);
}

} // namespace agv
} // namespace rmf_traffic

namespace std {

//==============================================================================
ostream& operator<<(
  ostream& os,
  const rmf_traffic::agv::Planner::CacheAudit& audit)
{
  os << "Cache sizes:"
     << "\n - DifferentialDrive: " << audit.differential_drive_planner_cache_size()
     << "\n - ShortestPath: " << audit.shortest_path_cache_size()
     << "\n - Euclidean: " << audit.euclidean_heuristic_cache_size();

  return os;
}

} // namespace std
