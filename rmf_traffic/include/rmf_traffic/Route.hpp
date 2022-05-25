/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__ROUTE_HPP
#define RMF_TRAFFIC__ROUTE_HPP

#include <rmf_traffic/Trajectory.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <optional>
#include <set>
#include <map>

namespace rmf_traffic {

//==============================================================================
using RouteId = uint64_t;
using ParticipantId = uint64_t;
using CheckpointId = uint64_t;
using PlanId = uint64_t;

/// Bundle of integers representing a dependency on a checkpoint within a
/// specific participant's plan.
struct Dependency
{
  uint64_t on_participant;
  uint64_t on_plan;
  uint64_t on_route;
  uint64_t on_checkpoint;

  /// Equality operator
  bool operator==(const Dependency& other) const;
};

using Dependencies = std::vector<Dependency>;

/// The checkpoint in the value waits for the checkpoint in the key
using DependsOnCheckpoint = std::map<CheckpointId, CheckpointId>;

/// The checkpoint dependencies relate to the route ID of the key
using DependsOnRoute = std::unordered_map<RouteId, DependsOnCheckpoint>;

//==============================================================================
/// Express a dependency on the plan of another traffic participant
class DependsOnPlan
{
public:

  /// Default constructor. There will be no dependency.
  DependsOnPlan();

  /// There will be a dependency on the specified plan.
  DependsOnPlan(PlanId plan, DependsOnRoute routes);

  /// Set the plan that there is a dependency on.
  DependsOnPlan& plan(std::optional<PlanId> plan);

  /// Get the plan that there is a dependency on.
  std::optional<PlanId> plan() const;

  /// Set the routes that there is a dependency on.
  DependsOnPlan& routes(DependsOnRoute routes);

  /// Get the routes that there is a dependency on.
  DependsOnRoute& routes();

  /// Get the routes that there is a dependency on.
  const DependsOnRoute& routes() const;

  struct Dependency
  {
    RouteId on_route;
    CheckpointId on_checkpoint;
  };

  /// Add a dependency
  DependsOnPlan& add_dependency(
    CheckpointId dependent_checkpoint,
    Dependency dependency);

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

/// Express a dependency on a participant
using DependsOnParticipant = std::unordered_map<ParticipantId, DependsOnPlan>;

//==============================================================================
/// A route on the schedule. This is used as a component of a schedule
/// participant's itinerary.
class Route
{
public:

  /// Constructor
  ///
  /// \param[in] map
  ///   The map that the trajectory is on
  ///
  /// \param[in] trajectory
  ///   The scheduled trajectory
  Route(
    std::string map,
    Trajectory trajectory);

  /// Set the map for this route
  Route& map(std::string value);

  /// Get the map for this route
  const std::string& map() const;

  /// Set the trajectory for this route
  Route& trajectory(Trajectory value);

  /// Get the trajectory for this route
  Trajectory& trajectory();

  /// Get the trajectory for this immutable route
  const Trajectory& trajectory() const;

  /// Set the checkpoints for this route. A checkpoint is a waypoint within this
  /// route which will explicitly trigger an traffic event update when it is
  /// reached.
  Route& checkpoints(std::set<uint64_t> value);

  /// Get the checkpoints for this route
  std::set<uint64_t>& checkpoints();

  /// Get the checkpoints for this immutable route
  const std::set<uint64_t>& checkpoints() const;

  /// Set the dependencies of the route
  Route& dependencies(DependsOnParticipant value);

  /// Get the dependencies of the route
  DependsOnParticipant& dependencies();

  /// Get the dependencies of the immutable route
  const DependsOnParticipant& dependencies() const;

  /// Tell this route that it has a dependency on the checkpoint of another
  /// participant's route.
  ///
  /// \param[in] dependent_checkpoint
  ///   The checkpoint inside of this route which has a dependency on the other
  ///   participant's route.
  ///
  /// \param[in] on_participant
  ///   The other participant which this route is depending on.
  ///
  /// \param[in] on_plan
  ///   The ID of the other participant's plan that this route is depending on.
  ///
  /// \param[in] on_route
  ///   The ID of the other participant's route that this robot is depending on.
  ///
  /// \param[in] on_checkpoint
  ///   The ID of the checkpoint
  Route& add_dependency(
    CheckpointId dependent_checkpoint,
    Dependency dependency);

  /// True if this route should ignore information about the given
  /// (participant, plan) pair. If this route has a dependency on a plan from
  /// this participant with a higher ID value, then this will return true.
  /// Otherwise it returns false.
  bool should_ignore(ParticipantId participant, PlanId plan) const;

  /// Get any dependencies that this route has on the given route of another
  /// participant.
  ///
  /// \param[in] on_participant
  ///   The ID of the other participant of interest
  ///
  /// \param[in] on_plan
  ///   The ID of the other participant's current plan
  ///
  /// \param[in] on_route
  ///   The ID of the other participant's route that is being considered
  ///
  /// \return A pointer to the relevant dependencies, if any exist. If there is
  /// no dependency relevant to the specified route of the participant, then
  /// this will be a nullptr.
  const DependsOnCheckpoint* check_dependencies(
    ParticipantId on_participant,
    PlanId on_plan,
    RouteId on_route) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

using RoutePtr = std::shared_ptr<Route>;
using ConstRoutePtr = std::shared_ptr<const Route>;

} // namespace rmf_traffic


#endif // RMF_TRAFFIC__SCHEDULDE__ROUTE_HPP
