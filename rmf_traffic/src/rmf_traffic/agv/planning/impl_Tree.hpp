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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_TREE_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_TREE_HPP

#include "Tree.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<typename E, typename C>
FrontierTemplate<E, C>::FrontierTemplate(Compare comparator)
: _comparator(std::move(comparator))
{
  // Do nothing
}

//==============================================================================
template<typename E, typename C>
auto FrontierTemplate<E, C>::pop() -> Element
{
  auto element = std::move(_storage.front());
  std::pop_heap(_storage.begin(), _storage.end(), _comparator);
  _storage.pop_back();
//  if (!std::is_heap(_storage.begin(), _storage.end(), _comparator))
//  {
//    const auto r_e = element->remaining_cost_estimate.value_or(std::numeric_limits<double>::infinity());
//    std::cout << "Removing element " << element->waypoint << " | " << element->current_cost + r_e
//              << " = " << element->current_cost << " + " << r_e << std::endl;

//    std::cout << "Remaining:\n";
//    std::optional<std::size_t> smaller_wp;
//    for (const auto& n : _storage)
//    {
//      const auto r_n = n->remaining_cost_estimate.value_or(std::numeric_limits<double>::infinity());
//      std::cout << " -- " << n->waypoint << " | " << n->current_cost + r_n
//                << " = " << n->current_cost << " + " << r_n << std::endl;

//      if (n->current_cost + r_n < element->current_cost + r_e)
//        smaller_wp = n->waypoint;
//    }

//    if (smaller_wp.has_value())
//      std::cout << *smaller_wp << " IS SMALLER!" << std::endl;
//    else
//      std::cout << "Popped the smallest" << std::endl;

//    throw std::runtime_error("pop: NOT A HEAP");
//  }
//  else
//    std::cout << "pop: is heap" << std::endl;

  return element;
}

//==============================================================================
template<typename E, typename C>
void FrontierTemplate<E, C>::push(Element new_element)
{
  _storage.push_back(std::move(new_element));
  std::push_heap(_storage.begin(), _storage.end(), _comparator);
//  if (!std::is_heap(_storage.begin(), _storage.end(), _comparator))
//    throw std::runtime_error("push: NOT A HEAP");
//  else
//    std::cout << "push: is heap" << std::endl;
}

//==============================================================================
template<typename E, typename C>
auto FrontierTemplate<E, C>::peek() const -> const Element*
{
  if (_storage.empty())
    return nullptr;

  return &_storage.front();
}

//==============================================================================
template<typename E, typename C>
bool FrontierTemplate<E, C>::empty() const
{
  return _storage.empty();
}

//==============================================================================
template<typename E, typename C>
void FrontierTemplate<E, C>::retarget(std::function<void(Element&)> transform)
{
  for (auto& element : _storage)
    transform(element);

  std::make_heap(_storage.begin(), _storage.end(), _comparator);
//  if (!std::is_heap(_storage.begin(), _storage.end(), _comparator))
//    throw std::runtime_error("retarget: NOT A HEAP");
//  else
//    std::cout << "retarget: is heap" << std::endl;
}

//==============================================================================
template<typename E, typename C>
auto FrontierTemplate<E, C>::storage() const -> const std::vector<Element>&
{
  return _storage;
}

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

  const auto top = _frontier.pop();
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
  const auto* peek = _frontier.peek();
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

//==============================================================================
template<typename ExpanderT>
void Tree<ExpanderT>::retarget(
  const Cache& cache,
  WaypointId new_target)
{
  _expander.retarget(cache, new_target, _frontier);
}

//==============================================================================
template<typename ExpanderT>
auto Tree<ExpanderT>::frontier() const -> const Frontier&
{
  return _frontier;
}

//==============================================================================
template<typename T, typename C>
LockedTree<T> TreeManager<T, C>::get_tree(
  WaypointId root_waypoint,
  const std::shared_ptr<const Supergraph>& graph,
  const typename T::Cache& heuristic,
  WaypointId target_waypoint)
{
  SpinLock lock(_tree_mutex);
  if (!_tree.has_value())
    _tree = Tree(root_waypoint, typename Tree::Expander(graph, heuristic, target_waypoint));

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
    const double full_cost = complement_node->current_cost;
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
                full_cost - current_node->current_cost + current_node->lane_cost,
                0.0, // placeholder which will get overwritten when retarget(~) is called
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

    // We iterate in reverse to minimize the amount of new frontier that gets
    // generated by inserting these.
    for (auto rit = new_nodes.rbegin(); rit != new_nodes.rend(); ++rit)
      _tree->insert(*rit);
  }

  _waiting_list.clear();
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
  return a.current_cost + b.current_cost - std::min(a.lane_cost, b.lane_cost);
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
  WaypointId root_waypoint,
  const std::shared_ptr<const Supergraph>& graph,
  const typename T::Cache& heuristic,
  WaypointId target_waypoint)
{
  SpinLock lock(mutex);
  return get_manager(manager_map, root_waypoint)
    ->second->get_tree(root_waypoint, graph, heuristic, target_waypoint);
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
    {
      lowest_cost = check;
//      std::cout << "New lowest overlap at " << visit->lane
//                << ": (" << visit->cost << ", " << visit->lane_cost << ") ("
//                << m_it->second->cost << ", " << m_it->second->lane_cost
//                << ") -> " << check << std::endl;
    }
  }

  return lowest_cost;
}

//==============================================================================
template<typename T, typename C>
inline std::optional<double> find_overlap(
  const T& forward,
  const C& reverse)
{
  const auto& f_visits = forward.all_visits();
  const auto& r_visits = reverse.all_visits();
  if (f_visits.size() < r_visits.size())
    return lowest_cost_overlap(f_visits, r_visits);

  return lowest_cost_overlap(r_visits, f_visits);
}
} // anonymous namespace

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_TREE_HPP
