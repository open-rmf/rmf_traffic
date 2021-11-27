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

#include <rmf_utils/math.hpp>

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<typename ExpanderT>
Tree<ExpanderT>::Tree(
  std::size_t initial_waypoint,
  Expander expander)
: _expander(std::move(expander))
{
  _expander.initialize(initial_waypoint, _frontier);
}

//==============================================================================
template<typename ExpanderT>
auto Tree<ExpanderT>::expand() -> NodePtr
{
  if (_frontier.empty())
    return nullptr;

  const auto top = _frontier.top();
  _frontier.pop();
  return _expander.expand(top, _frontier, _visited);
}

//==============================================================================
template<typename ExpanderT>
void Tree<ExpanderT>::insert(NodePtr node)
{
  _expander.expand(node, _frontier, _visited);
}

//==============================================================================
template<typename ExpanderT>
auto Tree<ExpanderT>::visited(LaneId lane_index) const -> NodePtr
{
  const auto it = _visited.find(lane_index);
  if (it == _visited.end())
    return nullptr;

  return it->second;
}

//==============================================================================
template<typename ExpanderT>
auto Tree<ExpanderT>::all_visits() const
-> const std::unordered_map<LaneId, NodePtr>&
{
  return _visited;
}

//==============================================================================
template<typename ExpanderT>
bool Tree<ExpanderT>::exhausted() const
{
  return _frontier.empty();
}

namespace {
//==============================================================================
template<std::size_t Traversal::*get_next_lane, typename NodePtrT>
void expand_traversals(
  const NodePtrT& top,
  DijkstraQueue<NodePtrT>& frontier,
  std::unordered_map<LaneId, NodePtrT>& visited,
  const std::shared_ptr<const Supergraph>& graph,
  const Traversals& traversals)
{
  for (const auto& traversal : traversals)
  {
    const auto next_lane = traversal.*get_next_lane;
    if (visited.count(next_lane) != 0)
    {
      // If this lane has already been visited, then we should not bother trying
      // to expand it.
      continue;
    }

    // TODO(MXG): We may be able to get a stronger heuristic here if we account
    // for irreversible robots and/or orientation constraints
    double rotational_cost = 0.0;
    std::optional<double> next_orientation;
    if (top->orientation.has_value())
    {
      const auto forward_init_angle = *top->orientation;
      const auto reverse_init_angle =
        rmf_utils::wrap_to_pi(forward_init_angle - M_PI);

      std::optional<double> minimum_angle;
      for (const auto init_angle : {forward_init_angle, reverse_init_angle})
      {
        for (const auto alternative : traversal.alternatives)
        {
          if (!alternative.has_value())
            continue;

          const auto next_angle = alternative->yaw;
          if (next_angle.has_value())
          {
            next_orientation = next_angle;
            const double check =
              std::abs(rmf_utils::wrap_to_pi(*next_angle - init_angle));

            if (!minimum_angle.has_value() || check < *minimum_angle)
              minimum_angle = check;
          }
        }
      }

      if (minimum_angle.has_value())
      {
        const auto& traits = graph->traits();
        rotational_cost = time::to_seconds(internal::estimate_rotation_time(
          traits.rotational().get_nominal_velocity(),
          traits.rotational().get_nominal_acceleration(),
          0.0, *minimum_angle,
          graph->options().rotation_thresh));
      }
    }

    frontier.push(
      std::make_shared<typename NodePtrT::element_type>(
        typename NodePtrT::element_type{
          next_lane,
          top->cost + traversal.best_time + rotational_cost,
          traversal.best_time,
          next_orientation,
          top
        }));
  }
}

//==============================================================================
template<std::size_t Traversal::*get_next_lane, typename NodePtrT>
void initialize_traversals(
  DijkstraQueue<NodePtrT>& frontier,
  const Traversals& traversals)
{
  for (const auto& traversal : traversals)
  {
    std::optional<double> orientation;
    for (const auto& alternative : traversal.alternatives)
    {
      if (alternative.has_value() && alternative->yaw.has_value())
      {
        orientation = *alternative->yaw;
        break;
      }
    }

    frontier.push(
      std::make_shared<typename NodePtrT::element_type>(
        typename NodePtrT::element_type{
          traversal.*get_next_lane,
          traversal.best_time,
          traversal.best_time,
          orientation,
          nullptr
        }));
  }
}
} // anonymous namespace

//==============================================================================
ForwardExpander::ForwardExpander(std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph))
{
  // Do nothing
}

//==============================================================================
ForwardNodePtr ForwardExpander::expand(
  const ForwardNodePtr& top,
  DijkstraQueue<ForwardNodePtr>& frontier,
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
  expand_traversals<&Traversal::finish_lane_index>(
    top,
    frontier,
    visited,
    _graph,
    *_graph->traversals_from(waypoint));

  return top;
}

//==============================================================================
void ForwardExpander::initialize(
  std::size_t waypoint_index,
  DijkstraQueue<ForwardNodePtr>& frontier) const
{
  const auto& traversals = *_graph->traversals_from(waypoint_index);
  initialize_traversals<&Traversal::finish_lane_index>(frontier, traversals);
}

//==============================================================================
ReverseExpander::ReverseExpander(std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph))
{
  // Do nothing
}

//==============================================================================
ReverseNodePtr ReverseExpander::expand(
  const ReverseNodePtr& top,
  DijkstraQueue<ReverseNodePtr>& frontier,
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
  expand_traversals<&Traversal::initial_lane_index>(
    top,
    frontier,
    visited,
    _graph,
    *_graph->traversals_into(waypoint));

  return top;
}

//==============================================================================
void ReverseExpander::initialize(
  std::size_t waypoint_index,
  DijkstraQueue<ReverseNodePtr>& frontier) const
{
  const auto& traversals = *_graph->traversals_into(waypoint_index);
  initialize_traversals<&Traversal::initial_lane_index>(frontier, traversals);
}

//==============================================================================
template<typename T, typename C>
LockedTree<T> TreeManager<T, C>::get_tree(
  std::size_t waypoint,
  const std::shared_ptr<const Supergraph>& graph)
{
  SpinLock lock(_tree_mutex);
  if (!_tree.has_value())
    _tree = Tree(waypoint, graph);

  _process_waiting_list();
  return LockedTree<T>{&(*_tree), std::move(lock)};
}

//==============================================================================
template<typename T, typename C>
void TreeManager<T, C>::add_to_waiting(ComplementNodePtr node)
{
  SpinLock lock(_waiting_list_mutex);
  _waiting_list.emplace_back(std::move(node));
}

//==============================================================================
template<typename T, typename C>
void TreeManager<T, C>::_process_waiting_list()
{
  SpinLock lock(_waiting_list_mutex);

  for (const auto& complement_node : _waiting_list)
  {
    const double full_cost = complement_node->cost;
    std::vector<NodePtr> new_nodes;
    NodePtr parent_node = nullptr;
    ComplementNodePtr current_node = complement_node;
    while (current_node)
    {
      if (const auto existing_node = _tree->visited(current_node->lane))
      {
        parent_node = existing_node;
      }
      else
      {
        auto new_node = std::make_shared<Node>(
              Node{
                current_node->lane,
                full_cost - current_node->cost + current_node->lane_cost,
                current_node->lane_cost,
                current_node->orientation,
                parent_node
              });

        parent_node = new_node;
        new_nodes.emplace_back(std::move(new_node));
      }

      current_node = current_node->parent;
    }

    // We iterate in reverse to minimize the amount of new frontier that gets
    // generated by inserting these.
    for (auto rit = new_nodes.rbegin(); rit != new_nodes.rend(); ++rit)
      _tree->insert(*rit);
  }

  _waiting_list.clear();
}

//==============================================================================
MinimalTravelHeuristic::MinimalTravelHeuristic(
  std::shared_ptr<const Supergraph> graph)
: _graph(std::move(graph))
{
  // Do nothing
}

namespace {
//==============================================================================
template<typename A, typename B>
double combine_costs(const A& a, const B& b)
{
  // The cost of boths nodes contains the cost of crossing the lane where their
  // states intersect. However, one of the lane costs might be greater than the
  // other because it may be going across multiple lanes instead of only going
  // down one lane. We subtract the lower of the two lane costs because the
  // lower lane cost would be leaving a gap in the overall path.
  return a.cost + b.cost - std::min(a.lane_cost, b.lane_cost);
}

//==============================================================================
template<typename T, typename C>
typename TreeManagerMap<T, C>::iterator get_manager(
  TreeManagerMap<T, C>& manager_map,
  WaypointId waypoint)
{
  const auto insertion = manager_map.insert({waypoint, nullptr});
  if (insertion.second)
    insertion.first->second = std::make_unique<TreeManager<T, C>>();

  return insertion.first;
}

//==============================================================================
template<typename T, typename C>
LockedTree<T> lock_tree(
  std::atomic_bool& mutex,
  TreeManagerMap<T, C>& manager_map,
  WaypointId waypoint,
  const std::shared_ptr<const Supergraph>& graph)
{
  SpinLock lock(mutex);
  return get_manager(manager_map, waypoint)
    ->second->get_tree(waypoint, graph);
}

//==============================================================================
template<typename A, typename B>
std::optional<double> lowest_cost_overlap(
  const A& fewer_visits,
  const B& more_visits)
{
  std::optional<double> lowest_cost;
  for (const auto& [_, visit] : fewer_visits)
  {
    const auto m_it = more_visits.find(visit->lane);
    if (m_it == more_visits.end())
      continue;

    const auto check = combine_costs(*visit, *m_it->second);
    if (!lowest_cost.has_value() || check < *lowest_cost)
      lowest_cost = check;
  }

  return lowest_cost;
}

//==============================================================================
std::optional<double> find_overlap(
  const ForwardTree& forward,
  const ReverseTree& reverse)
{
  const auto& f_visits = forward.all_visits();
  const auto& r_visits = reverse.all_visits();
  if (f_visits.size() < r_visits.size())
    return lowest_cost_overlap(f_visits, r_visits);

  return lowest_cost_overlap(r_visits, f_visits);
}
} // anonymous namespace

//==============================================================================
std::optional<double> MinimalTravelHeuristic::get(
  WaypointId start, WaypointId finish) const
{
  if (const auto solution = _check_for_solution(start, finish))
    return *solution;

  LockedTree<ForwardTree> forward =
    lock_tree(_forward_mutex, _forward, start, _graph);

  LockedTree<ReverseTree> reverse =
    lock_tree(_reverse_mutex, _reverse, finish, _graph);

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
  std::optional<double> result;
  std::vector<ForwardNodePtr> new_forwards;
  std::vector<ReverseNodePtr> new_reverses;
  {
    auto& forward = *forward_locked->tree;
    auto& reverse = *reverse_locked->tree;
    while (!(forward.exhausted() && reverse.exhausted()))
    {
      {
        const auto next_forward = forward.expand();
        if (next_forward)
        {
          if (const auto overlap = reverse.visited(next_forward->lane))
          {
            result = combine_costs(*next_forward, *overlap);
            break;
          }
        }
      }

      const auto next_reverse = reverse.expand();
      if (next_reverse)
      {
        if (const auto overlap = forward.visited(next_reverse->lane))
        {
          result = combine_costs(*next_reverse, *overlap);
          break;
        }
      }
    }
  }

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
