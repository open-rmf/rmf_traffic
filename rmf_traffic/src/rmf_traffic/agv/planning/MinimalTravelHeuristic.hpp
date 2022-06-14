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

class MinimumTravel : public DefaultForestSettings
{
public:

  using HeuristicCachePtr = std::shared_ptr<const ShortestPathHeuristic>;

  template<typename F>
  static bool exhausted(const F& frontier)
  {
    const auto* peek = frontier.peek();
    if (!peek)
      return true;

    // We always put expanded elements into the frontier even if they have no
    // prospect of reach the current target, because they might still matter for
    // reaching a different target during a different search. However we sort
    // those elements to all be in the bottom of the frontier queue. If one of
    // those elements reaches the top of the queue, then we know we have run out
    // of nodes that are worth exploring for this search.
    return !(*peek)->remaining_cost_estimate.has_value();
  }

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

  class ForwardExpander
    : public Expander<ForwardNode, HeuristicCachePtr, OptionalCompare>
  {
  public:

    ForwardExpander(
      std::shared_ptr<const Supergraph> graph,
      const HeuristicCachePtr& cache,
      const WaypointId target);

    ForwardNodePtr expand(
      const ForwardNodePtr& top,
      Frontier& frontier,
      std::unordered_map<LaneId, ForwardNodePtr>& visited) const final;

    void initialize(
      std::size_t waypoint,
      Frontier& frontier) const final;

    void retarget(
      const HeuristicCachePtr& cache,
      WaypointId new_target,
      Frontier& frontier) final;

    bool exhausted(const Frontier& frontier) const final;

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

  struct GetKey
  {
    LaneId operator()(const ForwardNodePtr& node) const
    {
      return node->lane;
    }

    LaneId operator()(const ReverseNodePtr& node) const
    {
      return node->lane;
    }
  };

  class ReverseExpander
    : public Expander<ReverseNode, HeuristicCachePtr, OptionalCompare>
  {
  public:

    ReverseExpander(
      std::shared_ptr<const Supergraph> graph,
      const HeuristicCachePtr& cache,
      const WaypointId target);

    ReverseNodePtr expand(
      const ReverseNodePtr& top,
      Frontier& frontier,
      std::unordered_map<LaneId, ReverseNodePtr>& visited) const final;

    void initialize(
      std::size_t waypoint,
      Frontier& frontier) const final;

    void retarget(
      const HeuristicCachePtr& cache,
      WaypointId new_target,
      Frontier& frontier) final;

    bool exhausted(const Frontier& frontier) const final;

  private:
    std::shared_ptr<const Supergraph> _graph;
    std::function<std::optional<double>(WaypointId)> _heuristic;
  };

  using ReverseTree = Tree<ReverseExpander>;

  using ForwardTreeManager = TreeManager<ForwardTree, ReverseTree>;
  using ReverseTreeManager = TreeManager<ReverseTree, ForwardTree>;

  using ForwardTreeManagerMap = TreeManagerMap<ForwardTree, ReverseTree>;
  using ReverseTreeManagerMap = TreeManagerMap<ReverseTree, ForwardTree>;

  template<typename T, typename C>
  static std::vector<typename T::NodePtr> flip_node(
    C current_node,
    const T& tree)
  {
    std::vector<typename T::NodePtr> new_nodes;
    const double full_cost = current_node->current_cost;
    typename T::NodePtr parent_node = nullptr;
    while (current_node)
    {
      if (const auto existing_node = tree.visited(current_node->lane))
      {
        parent_node = existing_node;
      }
      else
      {
        auto new_node = std::make_shared<typename T::Node>(
          typename T::Node{
            current_node->lane,
            full_cost - current_node->current_cost + current_node->lane_cost,
            // this placeholder will get overwritten when retarget(~) is called
            0.0,
            current_node->lane_cost,
            // We switch the waypoint and the complement_waypoint here
            // because we are reversing the type of node
            current_node->complement_waypoint,
            current_node->waypoint,
            current_node->orientation,
            parent_node
          });

        parent_node = new_node;
        new_nodes.emplace_back(std::move(new_node));
      }

      current_node = current_node->parent;
    }

    return new_nodes;
  }
};

//==============================================================================
class MinimalTravelHeuristic : public BidirectionalForest<MinimumTravel>
{
public:

  MinimalTravelHeuristic(
    std::shared_ptr<const Supergraph> graph);
};

//==============================================================================
using ConstMinimalTravelHeuristicPtr =
  std::shared_ptr<const MinimalTravelHeuristic>;

//==============================================================================
inline ForestSolution combine_paths(
  const MinimumTravel::ForwardNode& a,
  const MinimumTravel::ReverseNode& b)
{
  // The cost of boths nodes contains the cost of crossing the lane where their
  // states intersect. However, one of the lane costs might be greater than the
  // other because it may be going across multiple lanes instead of only going
  // down one lane. We subtract the lower of the two lane costs because the
  // lower lane cost would be leaving a gap in the overall path.
  const double cost =
    a.current_cost + b.current_cost - std::min(a.lane_cost, b.lane_cost);

  std::vector<std::size_t> path;

  // Do not add the first waypoint of either a or b because those will both be
  // covered by adding the waypoints of a's parent and b's parent.
  auto f_node = a.parent;
  while (f_node)
  {
    path.push_back(f_node->waypoint);
    f_node = f_node->parent;
  }

  std::reverse(path.begin(), path.end());

  auto r_node = b.parent;
  while (r_node)
  {
    path.push_back(r_node->waypoint);
    r_node = r_node->parent;
  }

  return ForestSolution{cost, std::move(path)};
}

//==============================================================================
inline ForestSolution combine_paths(
  const MinimumTravel::ReverseNode& b,
  const MinimumTravel::ForwardNode& a)
{
  return combine_paths(a, b);
}

//==============================================================================
inline std::vector<MinimumTravel::ReverseNodePtr> flip_node(
  MinimumTravel::ForwardNodePtr current_node,
  const MinimumTravel::ReverseTree& tree)
{
  return MinimumTravel::flip_node(current_node, tree);
}

//==============================================================================
inline std::vector<MinimumTravel::ForwardNodePtr> flip_node(
  MinimumTravel::ReverseNodePtr current_node,
  const MinimumTravel::ForwardTree& tree)
{
  return MinimumTravel::flip_node(current_node, tree);
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#include "impl_MinimalTravelHeuristic.hpp"

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP
