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

#include "MinimalTravelHeuristic.hpp"
#include "../internal_Interpolate.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
MinimumTravel::ForwardExpander::ForwardExpander(
  std::shared_ptr<const Supergraph> graph,
  const HeuristicCachePtr& cache,
  const WaypointId target)
: _graph(std::move(graph))
{
  _heuristic = [cache, target](WaypointId from)
    {
      return cache->get_cost(from, target);
    };
}

//==============================================================================
MinimumTravel::ForwardNodePtr MinimumTravel::ForwardExpander::expand(
  const ForwardNodePtr& top,
  Frontier& frontier,
  std::unordered_map<LaneId, ForwardNodePtr>& visited) const
{
  const auto was_inserted = visited.insert({top->lane, top}).second;
  if (!was_inserted)
  {
    // If this node was not successfully inserted into the visitors then that
    // means the lane was already visited and no expansion should occur because
    // a better cost was found for it.
    return nullptr;
  }

  const auto& top_lane = _graph->original().lanes.at(top->lane);
  const auto waypoint = top_lane.exit().waypoint_index();
  expand_traversals<
    & Traversal::finish_lane_index,
    & Traversal::finish_waypoint_index,
    & Traversal::initial_waypoint_index>(
    top,
    frontier,
    visited,
    _graph,
    _heuristic,
    *_graph->traversals_from(waypoint));

  if (!top->remaining_cost_estimate.has_value())
    return nullptr;

  return top;
}

//==============================================================================
void MinimumTravel::ForwardExpander::initialize(
  std::size_t waypoint_index,
  Frontier& frontier) const
{
  const auto& traversals = *_graph->traversals_from(waypoint_index);
  initialize_traversals<
    & Traversal::finish_lane_index,
    & Traversal::finish_waypoint_index,
    & Traversal::initial_waypoint_index>(
    frontier, _heuristic, traversals);
}

//==============================================================================
void MinimumTravel::ForwardExpander::retarget(
  const HeuristicCachePtr& cache,
  WaypointId new_target,
  Frontier& frontier)
{
  _heuristic = [cache, new_target](WaypointId from)
    {
      return cache->get_cost(from, new_target);
    };

  // It is okay to capture by reference because the lambda only gets used within
  // the scope of this function. The retarget(~) function does not store it for
  // later.
  frontier.retarget(
    [&](const std::shared_ptr<ForwardNode>& element)
    {
      element->remaining_cost_estimate = _heuristic(element->waypoint);
    });
}

//==============================================================================
bool MinimumTravel::ForwardExpander::exhausted(const Frontier& frontier) const
{
  return MinimumTravel::exhausted(frontier);
}

//==============================================================================
MinimumTravel::ReverseExpander::ReverseExpander(
  std::shared_ptr<const Supergraph> graph,
  const HeuristicCachePtr& cache,
  WaypointId target)
: _graph(std::move(graph))
{
  _heuristic = [cache, target](WaypointId from)
    {
      return cache->get_cost(target, from);
    };
}

//==============================================================================
MinimumTravel::ReverseNodePtr MinimumTravel::ReverseExpander::expand(
  const ReverseNodePtr& top,
  Frontier& frontier,
  std::unordered_map<LaneId, ReverseNodePtr>& visited) const
{
  const auto insertion = visited.insert({top->lane, top});
  const auto was_inserted = insertion.second;
  if (!was_inserted)
  {
    // If this node was not successfully inserted into the visitors then that
    // means the lane was already visited and no expansion should occur because
    // a better cost was found for it.
    return nullptr;
  }

  const auto& top_lane = _graph->original().lanes.at(top->lane);
  const auto waypoint = top_lane.entry().waypoint_index();
  expand_traversals<
    & Traversal::initial_lane_index,
    & Traversal::initial_waypoint_index,
    & Traversal::finish_waypoint_index>(
    top,
    frontier,
    visited,
    _graph,
    _heuristic,
    *_graph->traversals_into(waypoint));

  if (!top->remaining_cost_estimate.has_value())
    return nullptr;

  return top;
}

//==============================================================================
void MinimumTravel::ReverseExpander::initialize(std::size_t waypoint_index,
  Frontier& frontier) const
{
  const auto& traversals = *_graph->traversals_into(waypoint_index);
  initialize_traversals<
    & Traversal::initial_lane_index,
    & Traversal::initial_waypoint_index,
    & Traversal::finish_waypoint_index>(
    frontier, _heuristic, traversals);
}

//==============================================================================
void MinimumTravel::ReverseExpander::retarget(
  const HeuristicCachePtr& cache,
  WaypointId new_target,
  Frontier& frontier)
{
  _heuristic = [cache, new_target](WaypointId from)
    {
      return cache->get_cost(new_target, from);
    };

  // It is okay to capture by reference because the lambda only gets used within
  // the scope of this function. The retarget(~) function does not store it for
  // later.
  frontier.retarget(
    [&](const std::shared_ptr<ReverseNode>& element)
    {
      element->remaining_cost_estimate = _heuristic(element->waypoint);
    });
}

//==============================================================================
bool MinimumTravel::ReverseExpander::exhausted(const Frontier& frontier) const
{
  return MinimumTravel::exhausted(frontier);
}

//==============================================================================
MinimalTravelHeuristic::MinimalTravelHeuristic(
  std::shared_ptr<const Supergraph> graph)
: BidirectionalForest<MinimumTravel>(
    graph, std::make_shared<ShortestPathHeuristic>(graph))
{
  // Do nothing
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
