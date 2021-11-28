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

#include <iostream>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
MinimumTravel::ForwardExpander::ForwardExpander(
  std::shared_ptr<const Supergraph> graph,
  const ChildHeuristicManagerMapPtr& cache,
  const WaypointId target)
: _graph(std::move(graph))
{
  _heuristic = [cache = cache->get(target)->get()](WaypointId from)
    {
      return cache.get(from);
    };
}

//==============================================================================
MinimumTravel::ForwardNodePtr MinimumTravel::ForwardExpander::expand(
  const ForwardNodePtr& top,
  Frontier& frontier,
  std::unordered_map<LaneId, ForwardNodePtr>& visited) const
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
  const auto waypoint = top_lane.exit().waypoint_index();
  expand_traversals<
    &Traversal::finish_lane_index,
    &Traversal::finish_waypoint_index,
    &Traversal::initial_waypoint_index>(
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
void MinimumTravel::ForwardExpander::initialize(std::size_t waypoint_index,
  Frontier& frontier) const
{
  const auto& traversals = *_graph->traversals_from(waypoint_index);
  initialize_traversals<
    &Traversal::finish_lane_index,
    &Traversal::finish_waypoint_index,
    &Traversal::initial_waypoint_index>(
      frontier, _heuristic, traversals);
}

//==============================================================================
void MinimumTravel::ForwardExpander::retarget(
  const ChildHeuristicManagerMapPtr& cache,
  WaypointId new_target,
  Frontier& frontier)
{
  _heuristic = [cache = cache->get(new_target)->get()](WaypointId from)
    {
      return cache.get(from);
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
MinimumTravel::ReverseExpander::ReverseExpander(
  std::shared_ptr<const Supergraph> graph,
  const ChildHeuristicManagerMapPtr& heuristic,
  WaypointId target)
: _graph(std::move(graph))
{
  _heuristic = [cache = heuristic, target](WaypointId from)
    {
      return cache->get(from)->get().get(target);
    };
}

//==============================================================================
MinimumTravel::ReverseNodePtr MinimumTravel::ReverseExpander::expand(
  const ReverseNodePtr& top,
  Frontier& frontier,
  std::unordered_map<LaneId, ReverseNodePtr>& visited) const
{
//  const auto& lane = _graph->original().lanes[top->lane];
//  std::cout << "Selecting " << lane.entry().waypoint_index() << " -> "
//            << lane.exit().waypoint_index() << " (" << top->cost << ":" << top->lane_cost << ") for expansion";
//  if (top->parent)
//  {
//    const auto& parent_lane = _graph->original().lanes[top->parent->lane];
//    std::cout << " | parent: " << parent_lane.entry().waypoint_index() << " -> "
//              << parent_lane.exit().waypoint_index() << std::endl;
//  }
//  else
//  {
//    std::cout << " | no parent" << std::endl;
//  }

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
    &Traversal::initial_lane_index,
    &Traversal::initial_waypoint_index,
    &Traversal::finish_waypoint_index>(
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
//  std::cout << "Initial reverse traversals:" << std::endl;
//  for (const auto& t : traversals)
//  {
//    std::cout << _graph->original().lanes[t.initial_lane_index].entry().waypoint_index()
//        << " -> " << _graph->original().lanes[t.finish_lane_index].exit().waypoint_index()
//        << "(" << t.best_time << ")" << std::endl;
//  }

  initialize_traversals<
    &Traversal::initial_lane_index,
    &Traversal::initial_waypoint_index,
    &Traversal::finish_waypoint_index>(
      frontier, _heuristic, traversals);
}

//==============================================================================
void MinimumTravel::ReverseExpander::retarget(
  const ChildHeuristicManagerMapPtr& cache,
  WaypointId new_target,
  Frontier& frontier)
{
  _heuristic = [cache = std::move(cache), new_target](WaypointId from)
    {
      return cache->get(from)->get().get(new_target);
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
MinimalTravelHeuristic::MinimalTravelHeuristic(
  std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph)),
  _heuristic_cache(std::make_shared<ChildHeuristicManagerMap>(std::make_shared<ChildHeuristicFactory>(_graph)))
{
  // Do nothing
}

//==============================================================================
class Timer
{
public:
  Timer(std::string purpose)
  : _purpose(std::move(purpose))
  {
    _start = std::chrono::steady_clock::now();
  }

  ~Timer()
  {
    const auto finish = std::chrono::steady_clock::now();
    std::cout << _purpose << ": " << rmf_traffic::time::to_seconds(finish - _start) << std::endl;
  }

private:
  std::string _purpose;
  std::chrono::steady_clock::time_point _start;
};

//==============================================================================
std::string waypoint_name(std::size_t index, const std::shared_ptr<const Supergraph>& graph)
{
  if (const auto* check = graph->original().waypoints[index].name())
    return std::to_string(index) + " [" + *check + "]";

  return std::to_string(index);
}

//==============================================================================
std::optional<double> MinimalTravelHeuristic::get(
  WaypointId start, WaypointId finish) const
{
//  std::cout << " ------ \nGetting heuristic for "  << start << " -> " << finish << std::endl;
//  Timer timer("Heuristic for " + waypoint_name(start, _graph)
//              + " -> " + waypoint_name(finish, _graph));

  if (start == finish)
    return 0.0;

  if (const auto solution = _check_for_solution(start, finish))
    return *solution;

  LockedTree<ForwardTree> forward =
    lock_tree(_forward_mutex, _forward, start, _graph, _heuristic_cache, finish);

  LockedTree<ReverseTree> reverse =
    lock_tree(_reverse_mutex, _reverse, finish, _graph, _heuristic_cache, start);

  if (const auto overlap = find_overlap(*forward.tree, *reverse.tree))
  {
    SpinLock lock(_solutions_mutex);
    _solutions[start][finish] = *overlap;
    return *overlap;
  }

  return _search(start, std::move(forward), finish, std::move(reverse));
}

//==============================================================================
std::optional<double> MinimalTravelHeuristic::_check_for_solution(
  WaypointId start, WaypointId finish) const
{
  SpinLock lock(_solutions_mutex);
  if (const auto s_it = _solutions.find(start); s_it != _solutions.end())
  {
    const auto finish_map = s_it->second;
    if (const auto f_it = finish_map.find(finish); f_it != finish_map.end())
      return f_it->second;
  }

  return std::nullopt;
}

//==============================================================================
std::optional<double> MinimalTravelHeuristic::_search(
  WaypointId start,
  std::optional<LockedTree<ForwardTree>> forward_locked,
  WaypointId finish,
  std::optional<LockedTree<ReverseTree>> reverse_locked) const
{
//  std::cout << "Searching for solution to " << start << " -> " << finish << std::endl;
//  Timer timer("Whole search");

  std::optional<double> result;
  std::vector<ForwardNodePtr> new_forwards;
  std::vector<ReverseNodePtr> new_reverses;
  {
    auto& forward = *forward_locked->tree;
    {
//      Timer("retarget forward");
      forward.retarget(_heuristic_cache, finish);
    }

    auto& reverse = *reverse_locked->tree;
    {
//      Timer("retarget reverse");
      reverse.retarget(_heuristic_cache, start);
    }

    while (!(forward.exhausted() && reverse.exhausted()))
    {
      {
        const auto next_forward = forward.expand();
        if (next_forward)
        {
          if (const auto overlap = reverse.visited(next_forward->lane))
          {
            result = combine_costs(*next_forward, *overlap);

//            const auto wp = _graph->original().lanes[next_forward->lane].exit().waypoint_index();
//            std::cout << start << " -> " << finish
//                      << " | Forward met at " << wp << " | " << next_forward->lane
//                      << ": (" << next_forward->current_cost << ", "
//                      << next_forward->lane_cost << ") ("
//                      << overlap->current_cost << ", " << overlap->lane_cost
//                      << ") -> " << *result << std::endl;

//            if (start == 4054 && finish == 1542)
//            {
//              auto f = next_forward;
//              std::cout << "Forward: ";
//              while (f)
//              {
//                const auto f_lane = _graph->original().lanes[f->lane];
//                const auto f_wp_0 = f->complement_waypoint;
//                const auto f_wp_1 = f->waypoint;
//                std::cout << f_wp_1 << " <- " << f_wp_0 << " (c " << f->current_cost << " : l " << f->lane_cost
//                          << " : r " << f->remaining_cost_estimate.value() << ") | ";
//                f = f->parent;
//              }
//              std::cout << "Begin" << std::endl;

//              std::cout << "Reverse: ";
//              auto r = overlap;
//              while (r)
//              {
//                const auto r_lane = _graph->original().lanes[r->lane];
//                const auto r_wp_0 = r->waypoint;
//                const auto r_wp_1 = r->complement_waypoint;
//                std::cout << r_wp_0 << " -> " << r_wp_1 << " (c " << r->current_cost << " : l " << r->lane_cost
//                          << " : r " << r->remaining_cost_estimate.value() << ") | ";
//                r = r->parent;
//              }
//              std::cout << "Finish" << std::endl;
//            }

            break;
          }
        }
      }

      {
        const auto next_reverse = reverse.expand();
        if (next_reverse)
        {
          if (const auto overlap = forward.visited(next_reverse->lane))
          {
            result = combine_costs(*next_reverse, *overlap);

//            const auto f_wp = _graph->original().lanes[next_reverse->lane].exit().waypoint_index();
//            std::cout << start << " -> " << finish
//                      << " | Reverse met at " << f_wp << " | " << next_reverse->lane
//                      << ": (" << next_reverse->current_cost << ", "
//                      << next_reverse->lane_cost << ") ("
//                      << overlap->current_cost << ", " << overlap->lane_cost
//                      << ") -> " << *result << std::endl;

//            if (start == 4054 && finish == 1542)
//            {
//              auto f = overlap;
//              std::cout << "Forward: ";
//              while (f)
//              {
//                const auto f_lane = _graph->original().lanes[f->lane];
//                const auto f_wp_0 = f->complement_waypoint;
//                const auto f_wp_1 = f->waypoint;
//                std::cout << f_wp_1 << " <- " << f_wp_0 << " (c " << f->current_cost << " : l " << f->lane_cost
//                          << " : r " << f->remaining_cost_estimate.value() << ") | ";
//                f = f->parent;
//              }
//              std::cout << "Begin" << std::endl;

//              std::cout << "Reverse: ";
//              auto r = next_reverse;
//              while (r)
//              {
//                const auto r_lane = _graph->original().lanes[r->lane];
//                const auto r_wp_0 = r->waypoint;
//                const auto r_wp_1 = r->complement_waypoint;
//                std::cout << r_wp_0 << " -> " << r_wp_1 << " (c " << r->current_cost << " : l " << r->lane_cost
//                          << " : r " << r->remaining_cost_estimate.value() << ") | ";
//                r = r->parent;
//              }
//              std::cout << "Finish" << std::endl;
//            }

            break;
          }
        }
      }
    }

//    std::cout << "Forward visits:";
//    for (const auto& visit : forward.all_visits())
//      std::cout << " " << visit.second->lane;
//    std::cout << "\n\nReverse visits:";
//    for (const auto& visit : reverse.all_visits())
//      std::cout << " " << visit.second->lane;
//    std::cout << "\n" << std::endl;
  }

//  if (!result.has_value())
//  {
//    std::cout << "FAILED TO FIND SOLUTION FOR " << start << " -> " << finish << std::endl;
//  }

  // Release the locks since we are done with these trees
  forward_locked = std::nullopt;
  reverse_locked = std::nullopt;

  // Now we use the results from this search to seed all other relevant trees,
  // in case they ever get used in the future.
  {
    SpinLock lock(_forward_mutex);
    for (const auto& node : new_reverses)
    {
      auto& manager = *get_manager(_forward, node->lane)->second;
      manager.add_to_waiting(node);
    }
  }

  {
    SpinLock lock(_reverse_mutex);
    for (const auto& node : new_forwards)
    {
      auto& manager = *get_manager(_reverse, node->lane)->second;
      manager.add_to_waiting(node);
    }
  }

  SpinLock lock(_solutions_mutex);
  _solutions[start][finish] = result;
  return result;
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
