/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP

#include "Supergraph.hpp"
#include "EuclideanHeuristic.hpp"
#include "ShortestPathHeuristic.hpp"
#include "Tree.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {
using ChildOfMinimalTravelHeuristic = ShortestPathHeuristic;
using ChildHeuristicFactory = ShortestPathHeuristicFactory;

//using ChildOfMinimalTravelHeuristic = EuclideanHeuristic;
//using ChildHeuristicFactory = EuclideanHeuristicFactory;

using ChildHeuristicManagerMap = CacheManagerMap<ChildHeuristicFactory>;
using ChildHeuristicManagerMapPtr = std::shared_ptr<const ChildHeuristicManagerMap>;

class MinimumTravel
{
public:
  struct ForwardNode;
  using ForwardNodePtr = std::shared_ptr<ForwardNode>;
  using ConstForwardNodePtr = std::shared_ptr<const ForwardNode>;
  struct ForwardNode
  {
    // This node represents the state of a robot that is at the end of this lane
    // and finished exiting it. The current_cost is how much time it took the
    // robot to reach the end of this state, starting from the initial waypoint of
    // the tree.
    LaneId lane;
    double current_cost;
    std::optional<double> remaining_cost_estimate;
    double lane_cost;
    WaypointId waypoint;
    WaypointId complement_waypoint;
    std::optional<double> orientation;
    ForwardNodePtr parent;
  };

  class ForwardExpander : public Expander<ForwardNode, ChildHeuristicManagerMapPtr>
  {
  public:

    ForwardExpander(
      std::shared_ptr<const Supergraph> graph,
      const ChildHeuristicManagerMapPtr& cache,
      const WaypointId target);

    ForwardNodePtr expand(
      const ForwardNodePtr& top,
      Frontier& frontier,
      std::unordered_map<LaneId, ForwardNodePtr>& visited) const final;

    void initialize(
      std::size_t waypoint,
      Frontier& frontier) const final;

    void retarget(
      const ChildHeuristicManagerMapPtr& cache,
      WaypointId new_target,
      Frontier& frontier) final;

  private:
    std::shared_ptr<const Supergraph> _graph;
    std::function<std::optional<double>(WaypointId)> _heuristic;
  };

  using ForwardTree = Tree<ForwardExpander>;

  struct ReverseNode;
  using ReverseNodePtr = std::shared_ptr<ReverseNode>;
  using ConstReverseNodePtr = std::shared_ptr<const ReverseNode>;
  struct ReverseNode
  {
    // This node represents the state of a robot that is at the start of this lane
    // and ready to enter it.
    LaneId lane;
    double current_cost;
    std::optional<double> remaining_cost_estimate;
    double lane_cost;
    WaypointId waypoint;
    WaypointId complement_waypoint;
    std::optional<double> orientation;
    ReverseNodePtr parent;
  };

  class ReverseExpander : public Expander<ReverseNode, ChildHeuristicManagerMapPtr>
  {
  public:

    ReverseExpander(
      std::shared_ptr<const Supergraph> graph,
      const ChildHeuristicManagerMapPtr& cache,
      const WaypointId target);

    ReverseNodePtr expand(
      const ReverseNodePtr& top,
      Frontier& frontier,
      std::unordered_map<LaneId, ReverseNodePtr>& visited) const final;

    void initialize(
      std::size_t waypoint,
      Frontier& frontier) const final;

    void retarget(
      const ChildHeuristicManagerMapPtr& cache,
      WaypointId new_target,
      Frontier& frontier) final;

  private:
    std::shared_ptr<const Supergraph> _graph;
    std::function<std::optional<double>(WaypointId)> _heuristic;
  };

  using ReverseTree = Tree<ReverseExpander>;

  using ForwardTreeManager = TreeManager<ForwardTree, ReverseTree>;
  using ReverseTreeManager = TreeManager<ReverseTree, ForwardTree>;

  using ForwardTreeManagerMap = TreeManagerMap<ForwardTree, ReverseTree>;
  using ReverseTreeManagerMap = TreeManagerMap<ReverseTree, ForwardTree>;
};

//==============================================================================
class MinimalTravelHeuristic : public MinimumTravel
{
public:

  MinimalTravelHeuristic(
    std::shared_ptr<const Supergraph> graph);

  std::optional<double> get(WaypointId start, WaypointId finish) const;

private:

  std::optional<double> _check_for_solution(
    WaypointId start, WaypointId finish) const;

  std::optional<double> _search(
    WaypointId start,
    std::optional<LockedTree<ForwardTree>> forward_locked,
    WaypointId finish,
    std::optional<LockedTree<ReverseTree>> reverse_locked) const;

  std::shared_ptr<const Supergraph> _graph;
  ChildHeuristicManagerMapPtr _heuristic_cache;

  mutable ForwardTreeManagerMap _forward;
  mutable std::atomic_bool _forward_mutex = false;

  mutable ReverseTreeManagerMap _reverse;
  mutable std::atomic_bool _reverse_mutex = false;

  using SolutionMap =
    std::unordered_map<
      WaypointId, std::unordered_map<WaypointId, std::optional<double>>>;
  mutable SolutionMap _solutions;
  mutable std::atomic_bool _solutions_mutex = false;
};

//==============================================================================
using ConstMinimalTravelHeuristicPtr =
  std::shared_ptr<const MinimalTravelHeuristic>;

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#include "impl_MinimalTravelHeuristic.hpp"

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP
