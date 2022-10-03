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

#include "ShortestPathHeuristic.hpp"
#include "a_star.hpp"

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<
  typename GetNextWaypoint,
  typename NodePtrT, typename C>
void expand_lane(
  const NodePtrT& top,
  FrontierTemplate<NodePtrT, C>& frontier,
  std::unordered_map<WaypointId, NodePtrT>& visited,
  const Graph::Implementation& g,
  const LaneClosure& closures,
  const double agent_max_speed,
  const std::vector<std::size_t>& lanes)
{
  const auto& wp_0 = g.waypoints[top->waypoint];
  const auto& p_0 = wp_0.get_location();
  for (const auto l : lanes)
  {
    if (closures.is_closed(l))
      continue;

    const auto& next_lane = g.lanes[l];
    const auto next_waypoint = GetNextWaypoint()(next_lane);
    if (visited.count(next_waypoint) != 0)
    {
      // If this waypoint has already been visited, then we should not bother
      // trying to expand it.
      continue;
    }

    const auto& wp_1 = g.waypoints[next_waypoint];
    const auto& p_1 = wp_1.get_location();

    const auto max_speed =
      next_lane.properties().speed_limit().value_or(agent_max_speed);

    double local_cost = (p_1 - p_0).norm()/max_speed;

    if (const auto* entry_event = next_lane.entry().event())
      local_cost += rmf_traffic::time::to_seconds(entry_event->duration());

    if (const auto* exit_event = next_lane.exit().event())
      local_cost += rmf_traffic::time::to_seconds(exit_event->duration());

    frontier.push(
      std::make_shared<typename NodePtrT::element_type>(
        typename NodePtrT::element_type{
          next_waypoint,
          top->current_cost + local_cost,
          top
        }));
  }
}

//==============================================================================
ShortestPath::ForwardExpander::ForwardExpander(
  std::shared_ptr<const Supergraph> graph,
  const HeuristicCachePtr&,
  const WaypointId)
: _graph(std::move(graph)),
  _max_speed(_graph->traits().linear().get_nominal_velocity())
{
  // Do nothing
}

//==============================================================================
struct ForwardGetNextWaypoint
{
  std::size_t operator()(const rmf_traffic::agv::Graph::Lane& lane) const
  {
    return lane.exit().waypoint_index();
  }
};

//==============================================================================
ShortestPath::ForwardNodePtr ShortestPath::ForwardExpander::expand(
  const ForwardNodePtr& top,
  Frontier& frontier,
  std::unordered_map<WaypointId, ForwardNodePtr>& visited) const
{
  const auto was_inserted = visited.insert({top->waypoint, top}).second;
  if (!was_inserted)
  {
    // If this node was not successfully inserted into the visitors then that
    // means the lane was already visited and no expansion should occur because
    // a better cost was found for it.
    return nullptr;
  }

  const auto& g = _graph->original();
  const auto& closures = _graph->closures();
  expand_lane<ForwardGetNextWaypoint>(
    top, frontier, visited, g, closures, _max_speed,
    g.lanes_from[top->waypoint]);

  return top;
}

//==============================================================================
void ShortestPath::ForwardExpander::initialize(
  std::size_t waypoint, Frontier& frontier) const
{
  frontier.push(
    std::make_shared<ForwardNode>(
      ForwardNode{
        waypoint,
        0.0,
        nullptr
      }));
}

//==============================================================================
void ShortestPath::ForwardExpander::retarget(
  const Cache&, WaypointId, Frontier&)
{
  // We do not retarget these trees
}

//==============================================================================
bool ShortestPath::ForwardExpander::exhausted(const Frontier& frontier) const
{
  return ShortestPath::exhausted(frontier);
}

//==============================================================================
ShortestPath::ReverseExpander::ReverseExpander(
  std::shared_ptr<const Supergraph> graph,
  const HeuristicCachePtr&,
  const WaypointId)
: _graph(std::move(graph)),
  _max_speed(_graph->traits().linear().get_nominal_velocity())
{
  // Do nothing
}

//==============================================================================
struct ReverseGetNextWaypoint
{
  std::size_t operator()(const rmf_traffic::agv::Graph::Lane& lane) const
  {
    return lane.entry().waypoint_index();
  }
};

//==============================================================================
ShortestPath::ReverseNodePtr ShortestPath::ReverseExpander::expand(
  const ReverseNodePtr& top,
  Frontier& frontier,
  std::unordered_map<WaypointId, ReverseNodePtr>& visited) const
{
  const auto was_inserted = visited.insert({top->waypoint, top}).second;
  if (!was_inserted)
  {
    // If this node was not successfully inserted into the visitors then that
    // means the lane was already visited and no expansion should occur because
    // a better cost was found for it.
    return nullptr;
  }

  const auto& g = _graph->original();
  const auto& closures = _graph->closures();
  expand_lane<ReverseGetNextWaypoint>(
    top, frontier, visited, g, closures, _max_speed,
    g.lanes_into[top->waypoint]);

  return top;
}

//==============================================================================
void ShortestPath::ReverseExpander::initialize(
  std::size_t waypoint,
  Frontier& frontier) const
{
  frontier.push(
    std::make_shared<ReverseNode>(
      ReverseNode{
        waypoint,
        0.0,
        nullptr
      }));
}

//==============================================================================
void ShortestPath::ReverseExpander::retarget(
  const Cache&, WaypointId, Frontier&)
{
  // We do not retarget these trees
}

//==============================================================================
bool ShortestPath::ReverseExpander::exhausted(const Frontier& frontier) const
{
  return ShortestPath::exhausted(frontier);
}

//==============================================================================
ShortestPathHeuristic::ShortestPathHeuristic(
  std::shared_ptr<const Supergraph> graph)
: BidirectionalForest<ShortestPath>(
    graph,
    std::make_shared<EuclideanHeuristicCacheMap>(
      std::make_shared<EuclideanHeuristicFactory>(graph)))
{
  // Do nothing
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
