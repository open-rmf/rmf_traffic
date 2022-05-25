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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP

#include "../internal_Graph.hpp"
#include "../internal_Interpolate.hpp"
#include "CacheManager.hpp"
#include "DifferentialDriveMap.hpp"

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>

#include <array>
#include <map>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
double calculate_cost(
  const rmf_traffic::Trajectory& traj,
  double traversal_cost_per_meter);

//==============================================================================
class Supergraph;

//==============================================================================
class DifferentialDriveConstraint
{
public:

  static const Eigen::Rotation2Dd R_pi;

  DifferentialDriveConstraint(
    const Eigen::Vector2d& forward,
    const bool reversible);

  std::array<std::optional<double>, 2> get_orientations(
    const Eigen::Vector2d& course_vector) const;

private:
  Eigen::Rotation2Dd R_f_inv;
  bool reversible;
};

//==============================================================================
struct Traversal
{
  std::size_t initial_lane_index;
  std::size_t finish_lane_index;
  std::size_t initial_waypoint_index;
  std::size_t finish_waypoint_index;
  Graph::Lane::EventPtr entry_event;
  Graph::Lane::EventPtr exit_event;
  std::vector<std::string> maps;
  std::vector<std::size_t> traversed_lanes;
  double best_cost;

  struct Alternative
  {
    double cost = 0.0;
    std::optional<double> yaw;

    using RouteFactoryFactory = DifferentialDriveMapTypes::RouteFactoryFactory;
    RouteFactoryFactory routes;
  };

  std::array<std::optional<Alternative>, 3> alternatives;
};
using Traversals = std::vector<Traversal>;
using ConstTraversalsPtr = std::shared_ptr<const Traversals>;

//==============================================================================
class TraversalFromGenerator
  : public Generator<std::unordered_map<std::size_t, ConstTraversalsPtr>>
{
public:

  TraversalFromGenerator(
    const std::shared_ptr<const Supergraph>& graph);

  ConstTraversalsPtr generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const final;

  struct Kinematics
  {
    Kinematics(
      const VehicleTraits& traits,
      const Interpolate::Options::Implementation& interpolate,
      double traversal_cost_per_meter);

    KinematicLimits limits;
    std::optional<DifferentialDriveConstraint> constraint;
    Interpolate::Options::Implementation interpolate;
    double traversal_cost_per_meter;
  };

private:
  std::weak_ptr<const Supergraph> _graph;
  Kinematics _kinematics;
};

//==============================================================================
using TraversalFromCache = Cache<TraversalFromGenerator>;

//==============================================================================
class TraversalIntoGenerator
  : public Generator<std::unordered_map<std::size_t, ConstTraversalsPtr>>
{
public:

  TraversalIntoGenerator(
    std::shared_ptr<const CacheManager<TraversalFromCache>> traversals_from,
    const std::shared_ptr<const Supergraph>& graph);

  ConstTraversalsPtr generate(
    const std::size_t& key,
    const Storage& old_items,
    Storage& new_items) const final;

private:
  std::shared_ptr<const CacheManager<TraversalFromCache>> _traversals_from;
  std::weak_ptr<const Supergraph> _graph;
};

//==============================================================================
using TraversalIntoCache = Cache<TraversalIntoGenerator>;

//==============================================================================
/// A Supergraph is derived from a regular Graph. It analyzes the vertices and
/// edges of a regular Graph and adds in new edges and extra information that
/// may be useful for planners. The underlying graph cannot be changed without
/// reconstructing the Supergraph instance.
class Supergraph : public std::enable_shared_from_this<Supergraph>
{
public:

  // TODO(MXG): We could consider moving some of this class's nested classes out
  // into the global scope to clean up the API here.

  static std::shared_ptr<const Supergraph> make(
    Graph::Implementation original,
    VehicleTraits traits,
    LaneClosure lane_closures,
    const Interpolate::Options::Implementation& interpolate,
    double traversal_cost_per_meter);

  const Graph::Implementation& original() const;
  const VehicleTraits& traits() const;
  const LaneClosure& closures() const;
  const Interpolate::Options::Implementation& options() const;
  double traversal_cost_per_meter() const;

  struct FloorChange
  {
    std::size_t lane;
  };
  using FloorChanges = std::vector<FloorChange>;
  using DestinationFloorMap = std::unordered_map<std::string, FloorChanges>;
  using FloorChangeMap = std::unordered_map<std::string, DestinationFloorMap>;
  /// Get the lanes that allow the floor to change. This is useful for
  /// identifying the bottlenecks for moving between different maps.
  const FloorChangeMap& floor_change() const;

  /// Get the continuous traversals that can be done from the given waypoint.
  /// This means traversals during which the robot does not need to stop or
  /// rotate.
  ConstTraversalsPtr traversals_from(std::size_t waypoint_index) const;

  /// Get the continuous traversals that can be done into the given waypoint.
  ConstTraversalsPtr traversals_into(
    std::size_t waypoint_index) const;

  using Entry = DifferentialDriveMapTypes::Entry;
  using EntryHash = DifferentialDriveMapTypes::EntryHash;

  class Entries
  {
  public:

    std::vector<Entry> relevant_entries(
      std::optional<double> orientation) const;

    Entries(
      std::map<double, Entry> angled_entries,
      std::optional<Entry> agnostic_entry);

  private:
    // These are entries that require a specific entry angle
    std::map<double, Entry> _angled_entries;

    // If there are any entries that don't require a specific entry angle, then
    // that will be captured here.
    std::optional<Entry> _agnostic_entry;

    std::size_t _total_entries;
  };
  using ConstEntriesPtr = std::shared_ptr<const Entries>;

  // Get all the ways to enter into the given waypoint
  ConstEntriesPtr entries_into(std::size_t waypoint_index) const;

  // Get the yaw of this lane+orientation combo
  std::optional<double> yaw_of(const Entry& entry) const;

  /// Get the keys for the DifferentialDriveHeuristic cache entries that are
  /// relevant for a given combination of start and goal conditions.
  DifferentialDriveKeySet keys_for(
    std::size_t start_waypoint_index,
    std::size_t goal_waypoint_index,
    std::optional<double> goal_orientation) const;

private:
  Supergraph(
    Graph::Implementation original,
    VehicleTraits traits,
    LaneClosure lane_closures,
    const Interpolate::Options::Implementation& interpolate,
    double traversal_cost_per_meter);

  Graph::Implementation _original;
  VehicleTraits _traits;
  LaneClosure _lane_closures;
  Interpolate::Options::Implementation _interpolate;
  double _traversal_cost_per_meter;
  FloorChangeMap _floor_changes;
  std::shared_ptr<const CacheManager<TraversalFromCache>> _traversals_from;
  std::shared_ptr<const CacheManager<TraversalIntoCache>> _traversals_into;
  std::optional<DifferentialDriveConstraint> _constraint;

  class EntriesGenerator
    : public Generator<std::unordered_map<std::size_t, ConstEntriesPtr>>
  {
  public:
    EntriesGenerator(
      const std::shared_ptr<const Supergraph>& graph);

    ConstEntriesPtr generate(
      const std::size_t& key,
      const Storage& old_items,
      Storage& new_items) const final;

  private:
    std::weak_ptr<const Supergraph> _graph;
    std::optional<DifferentialDriveConstraint> _constraint;
  };
  using EntriesCache = Cache<EntriesGenerator>;
  std::shared_ptr<const CacheManager<EntriesCache>> _entries_into_waypoint_cache;

  using LaneYawMap =
    std::unordered_map<Entry, std::optional<double>, EntryHash>;

  class LaneYawGenerator : public Generator<LaneYawMap>
  {
  public:
    LaneYawGenerator(
      const std::shared_ptr<const Supergraph>& graph);

    std::optional<double> generate(
      const Entry& key,
      const Storage& old_items,
      Storage& new_items) const final;

  private:
    std::weak_ptr<const Supergraph> _graph;
    std::optional<DifferentialDriveConstraint> _constraint;
  };
  using LaneYawCache = Cache<LaneYawGenerator>;
  std::shared_ptr<const CacheManager<LaneYawCache>> _lane_yaw_cache;
};

//==============================================================================
bool orientation_constraint_satisfied(
  const Eigen::Vector2d p,
  const double orientation,
  const Eigen::Vector2d course_vector,
  const rmf_traffic::agv::Graph::OrientationConstraint* constraint,
  const double rotation_thresh);

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__SUPERGRAPH_HPP
