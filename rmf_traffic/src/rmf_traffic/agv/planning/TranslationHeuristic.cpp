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

#include "TranslationHeuristic.hpp"
#include "a_star.hpp"

#include <queue>

#include <iostream>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
class TranslationExpander
{
public:

  struct Node;
  using NodePtr = std::shared_ptr<const Node>;

  struct Node
  {
    std::size_t waypoint;

    // For the remaining_cost_estimate we'll use the ShortestPathExpander
    double remaining_cost_estimate;
    double current_cost;
    NodePtr parent;

    // TODO(YV) std::vector<Route> route_from_parent;
  };

  using SearchQueue =
    std::priority_queue<
    NodePtr, std::vector<NodePtr>, SimpleCompare<NodePtr>>;

  bool quit(const NodePtr&, const SearchQueue&) const
  {
    return false;
  }

  bool is_finished(const NodePtr& top) const
  {
    if (top->waypoint == _goal)
      return true;

    return false;
  }

  void expand(const NodePtr& top, SearchQueue& queue)
  {
    // TODO(MXG): We could probably refactor this so that
    // Euclidean, ShortestPath, and Translation Heuristics
    // all share most of this implementation.

    const auto current_wp_index = top->waypoint;
    if (!_visited.insert(current_wp_index).second)
    {
      // This means we have already expanded from this waypoint before.
      // Expanding from here again is pointless because expanding from a more
      // costly parent cannot be better than expanding from a less costly one.
      return;
    }

    const auto current_cost = top->current_cost;
    const auto old_it = _old_items.find(current_wp_index);
    if (old_it != _old_items.end())
    {
      // If the current waypoint already has an entry in the old items, then we
      // can immediately create a node that brings it the rest of the way to the
      // goal with the best possible cost.
      const auto remaining_cost = old_it->second;
      if (!remaining_cost.has_value())
      {
        // If the old value is a nullopt, then this waypoint has no way to reach
        // the goal, so we should just discard it.
        return;
      }

      auto new_node = std::make_shared<Node>(
        Node{
          _goal,
          0.0,
          current_cost + remaining_cost.value(),
          top
        });

      queue.push(std::move(new_node));
      return;
    }

    const auto traversals = _graph->traversals_from(current_wp_index);
    assert(traversals);
    for (const auto& traversal : *traversals)
    {
      const auto next_waypoint_index = traversal.finish_waypoint_index;
      if (_visited.count(next_waypoint_index))
      {
        // If we have already expanded from this waypoint, then there is no
        // point in re-expanding to it.
        continue;
      }

      const auto remaining_cost_estimate = _heuristic.get(next_waypoint_index);
      if (!remaining_cost_estimate.has_value())
      {
        // If the heuristic for this waypoint is a nullopt, then there is no way
        // to reach the goal from this node. We should just skip this expansion.
        continue;
      }

      auto new_node = std::make_shared<Node>(
        Node{
          next_waypoint_index,
          remaining_cost_estimate.value(),
          current_cost + traversal.best_time,
          top
        });

      queue.push(std::move(new_node));
    }
  }

  TranslationExpander(
    std::size_t goal,
    const TranslationHeuristic::Storage& old_items,
    Cache<ShortestPathHeuristic> heuristic,
    std::shared_ptr<const Supergraph> graph)
  : _goal(goal),
    _old_items(old_items),
    _heuristic(heuristic),
    _graph(std::move(graph))
  {
    // Do nothing
  }

private:
  std::size_t _goal;
  const TranslationHeuristic::Storage& _old_items;
  Cache<ShortestPathHeuristic> _heuristic;
  std::shared_ptr<const Supergraph> _graph;
  std::unordered_set<std::size_t> _visited;
};

//==============================================================================
TranslationHeuristic::TranslationHeuristic(
  std::size_t goal,
  std::shared_ptr<const Supergraph> graph,
  CacheManagerPtr<ShortestPathHeuristic> heuristic)
: _goal(goal),
  _graph(std::move(graph)),
  _heuristic(std::move(heuristic))
{
  // Do nothing
}

//==============================================================================
namespace {
using Storage = TranslationHeuristic::Storage;

const TranslationExpander::NodePtr search(
  const std::size_t& key,
  const Storage& old_items,
  Storage& new_items,
  CacheManagerPtr<ShortestPathHeuristic> _heuristic,
  const std::size_t& _goal,
  std::shared_ptr<const Supergraph> _graph)
{
  auto heuristic = _heuristic->get();
  auto start_heuristic = heuristic.get(key);
  if (!start_heuristic.has_value())
  {
    // If the heuristic of this starting waypoint is a nullopt, then it is
    // impossible for the start to reach the goal.
    new_items.insert({key, std::nullopt});
    return nullptr;
  }

  TranslationExpander::SearchQueue queue;
  queue.push(
    std::make_shared<TranslationExpander::Node>(
      TranslationExpander::Node{
        key,
        start_heuristic.value(),
        0.0,
        nullptr
      }));

  TranslationExpander expander{
    _goal,
    old_items,
    std::move(heuristic),
    _graph
  };

  return a_star_search(expander, queue);
}

// TODO(YV) These functions are copied from DifferentialDrivePlanner.cpp.
// Move them into a common utils header.
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while (node)
  {
    std::cout << node->waypoint << "->";
    node_sequence.push_back(node);
    node = node->parent;
  }
  std::cout << std::endl;

  return node_sequence;
}

struct Indexing
{
  std::size_t itinerary_index;
  std::size_t trajectory_index;
};

// template<typename NodePtr>
// std::pair<std::vector<Route>, std::vector<Indexing>> reconstruct_routes(
//   const std::vector<NodePtr>& node_sequence,
//   rmf_utils::optional<rmf_traffic::Duration> span = rmf_utils::nullopt)
// {
//   if (node_sequence.size() == 1)
//   {
//     std::vector<Route> output;
//     // If there is only one node in the sequence, then it is a start node.
//     if (span)
//     {
//       // When performing a rollout, it is important that at least one route with
//       // two waypoints is provided. We use the span value to creating a
//       // stationary trajectory when the robot is already starting out at a
//       // holding point.

//       // TODO(MXG): Make a unit test for this situation
//       std::vector<Route> simple_route =
//         node_sequence.back()->route_from_parent;
//       if (simple_route.back().trajectory().size() < 2)
//       {
//         const auto& wp = simple_route.back().trajectory().back();
//         for (auto&  r : simple_route)
//         {
//           r.trajectory().insert(
//             wp.time() + *span,
//             wp.position(),
//             Eigen::Vector3d::Zero());
//         }
//       }

//       output = std::move(simple_route);
//     }

//     // When there is only one node, we should return an empty itinerary to
//     // indicate that the AGV does not need to go anywhere.
//     return {output, {}};
//   }

//   std::vector<Indexing> indexing;
//   indexing.resize(node_sequence.size());

//   assert(node_sequence.back()->start.has_value());

//   std::vector<Route> routes = node_sequence.back()->route_from_parent;
//   indexing.at(node_sequence.size()-1).itinerary_index = 0;
//   indexing.at(node_sequence.size()-1).trajectory_index = 0;

//   // We exclude the first node in the sequence, because it contains an empty
//   // route which is not helpful.
//   const std::size_t N = node_sequence.size();
//   const std::size_t stop = node_sequence.size()-1;
//   for (std::size_t i = 0; i < stop; ++i)
//   {
//     const std::size_t n = N-2-i;
//     const auto& node = node_sequence.at(n);
//     auto& index = indexing.at(n);

//     for (const Route& next_route : node->route_from_parent)
//     {
//       Route& last_route = routes.back();
//       if (next_route.map() == last_route.map())
//       {
//         for (const auto& waypoint : next_route.trajectory())
//         {
//           last_route.trajectory().insert(waypoint);
//         }
//       }
//       else
//       {
//         routes.push_back(next_route);
//       }

//       // We will take note of the itinerary and trajectory indices here
//       index.itinerary_index = routes.size() - 1;
//       assert(!routes.back().trajectory().empty());
//       index.trajectory_index = routes.back().trajectory().size() - 1;
//     }

//     assert(routes.back().trajectory().back().time() == node->time);
//   }

//   // Throw away any routes that have less than two waypoints.
//   const auto r_it = std::remove_if(
//     routes.begin(), routes.end(),
//     [](const auto& r) { return r.trajectory().size() < 2; });
//   routes.erase(r_it, routes.end());

//   return {routes, indexing};
// }

} // anonymous namespace

//==============================================================================
std::optional<PlanData> TranslationHeuristic::translation_solve(
  const std::size_t& key,
  const Storage& old_items,
  Storage& new_items) const
{
  const TranslationExpander::NodePtr solution = search(
    key, old_items, new_items, _heuristic, _goal, _graph);

  if (!solution)
  {
    // This means there is no way to move to the goal from the start waypoint
    new_items.insert({key, std::nullopt});
    return std::nullopt;
  }

  // struct PlanData
  // {
  //   std::vector<Route> routes;
  //   std::vector<agv::Plan::Waypoint> waypoints;
  //   agv::Planner::Start start;
  //   double cost;
  // };
  auto nodes = reconstruct_nodes(solution);
  // auto [routes, index] = reconstruct_routes(nodes);
  // auto waypoints = reconstruct_waypoints(
  //   nodes, index, _supergraph->original());
  // auto start = find_start(solution);

  return std::nullopt;
}

//==============================================================================
std::optional<double> TranslationHeuristic::generate(
  const std::size_t& key,
  const Storage& old_items,
  Storage& new_items) const
{
  const TranslationExpander::NodePtr solution = search(
    key, old_items, new_items, _heuristic, _goal, _graph);

  if (!solution)
  {
    // This means there is no way to move to the goal from the start waypoint
    new_items.insert({key, std::nullopt});
    return std::nullopt;
  }

  const double final_cost = solution->current_cost;
  auto node = solution;
  while (node)
  {
    // We can save the results for every waypoint that was used in this solution
    // because every segment of an optimal solution is an optimal solution
    // itself.
    new_items.insert({node->waypoint, final_cost - node->current_cost});
    node = node->parent;
  }

  return final_cost;
}

//==============================================================================
TranslationHeuristicFactory::TranslationHeuristicFactory(
  std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph)),
  _heuristic_cache(std::make_shared<ShortestPathHeuristicFactory>(_graph))
{
  // Do nothing
}

//==============================================================================
ConstTranslationHeuristicPtr TranslationHeuristicFactory::make(
  const std::size_t goal) const
{
  return std::make_shared<TranslationHeuristic>(
    goal, _graph, _heuristic_cache.get(goal));
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
