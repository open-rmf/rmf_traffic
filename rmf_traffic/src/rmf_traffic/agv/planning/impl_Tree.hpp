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

#include <iostream>

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
  return element;
}

//==============================================================================
template<typename E, typename C>
void FrontierTemplate<E, C>::push(Element new_element)
{
  _storage.push_back(std::move(new_element));
  std::push_heap(_storage.begin(), _storage.end(), _comparator);
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
  return _expander.exhausted(_frontier);
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
  {
    _tree = Tree(
      root_waypoint,
      typename Tree::Expander(graph, heuristic, target_waypoint));
  }

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
    const auto new_nodes = flip_node(complement_node, *_tree);

    // We iterate in reverse to minimize the amount of new frontier that gets
    // generated by inserting these.
    for (auto rit = new_nodes.rbegin(); rit != new_nodes.rend(); ++rit)
      _tree->insert(*rit);
  }

  _waiting_list.clear();
}

namespace {

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
template<typename T, typename A, typename B>
std::optional<ForestSolution> lowest_cost_overlap(
  const A& fewer_visits,
  const B& more_visits)
{
  using GetKey = typename T::GetKey;
  std::optional<ForestSolution> lowest_cost;
  for (const auto& [_, visit] : fewer_visits)
  {
    const auto m_it = more_visits.find(GetKey()(visit));
    if (m_it == more_visits.end())
      continue;

    const auto check = combine_paths(*visit, *m_it->second);
    if (!lowest_cost.has_value() || check.cost < lowest_cost->cost)
    {
      lowest_cost = check;
    }
  }

  return lowest_cost;
}

//==============================================================================
template<typename T, typename A, typename B>
inline std::optional<ForestSolution> find_overlap(
  const A& forward,
  const B& reverse)
{
  const auto& f_visits = forward.all_visits();
  const auto& r_visits = reverse.all_visits();
  if (f_visits.size() < r_visits.size())
    return lowest_cost_overlap<T>(f_visits, r_visits);

  return lowest_cost_overlap<T>(r_visits, f_visits);
}
} // anonymous namespace


//==============================================================================
template<typename T>
class Timer
{
public:
  Timer(std::string purpose, std::chrono::steady_clock::duration& max)
  : _purpose(std::move(purpose)),
    _max(&max)
  {
    _start = std::chrono::steady_clock::now();
  }

  ~Timer()
  {
    const auto finish = std::chrono::steady_clock::now();
    const auto duration = finish - _start;
    if constexpr (T::print_timers)
    {
      std::cout << _purpose << ": "
                << rmf_traffic::time::to_seconds(duration) << std::endl;
    }

    *_max = std::max(*_max, duration);
  }

private:
  std::string _purpose;
  std::chrono::steady_clock::time_point _start;
  std::chrono::steady_clock::duration* _max;
};

//==============================================================================
inline std::string waypoint_name(
  std::size_t index,
  const std::shared_ptr<const Supergraph>& graph)
{
  if (const auto* check = graph->original().waypoints[index].name())
    return std::to_string(index) + " [" + *check + "]";

  return std::to_string(index);
}

//==============================================================================
template<typename T>
BidirectionalForest<T>::BidirectionalForest(
  std::shared_ptr<const Supergraph> graph,
  Cache cache)
: _graph(std::move(graph)),
  _heuristic_cache(std::move(cache)),
  _identity_solution(std::make_shared<ForestSolution>(ForestSolution{0.0, {}}))
{
  // Do nothing
}

//==============================================================================
template<typename T>
ConstForestSolutionPtr BidirectionalForest<T>::get(
  WaypointId start, WaypointId finish) const
{
  if constexpr (T::count_usage)
    ++_usage_count;

  if (start == finish)
    return _identity_solution;

  {
    const auto solution = _check_for_solution(start, finish);
    if (solution.has_value())
      return *solution;
  }

  LockedTree<ForwardTree> forward = lock_tree(
    _forward_mutex, _forward, start, _graph, _heuristic_cache, finish);

  LockedTree<ReverseTree> reverse = lock_tree(
    _reverse_mutex, _reverse, finish, _graph, _heuristic_cache, start);

  if (const auto overlap = find_overlap<T>(*forward.tree, *reverse.tree))
  {
    SpinLock lock(_solutions_mutex);
    auto solution = std::make_shared<ForestSolution>(*overlap);
    _solutions[start][finish] = solution;
    return solution;
  }

  return _search(start, std::move(forward), finish, std::move(reverse));
}

//==============================================================================
template<typename T>
std::optional<double> BidirectionalForest<T>::get_cost(
  WaypointId start, WaypointId finish) const
{
  if (const auto solution = get(start, finish))
    return solution->cost;

  return std::nullopt;
}

//==============================================================================
template<typename T>
BidirectionalForest<T>::~BidirectionalForest()
{
  if constexpr (T::count_usage)
  {
    std::cout << typeid(T).name() << " used: "
              << _usage_count << " | searched: " << _search_count << std::endl;
  }

  if constexpr (T::max_timer)
  {
    std::cout << typeid(T).name() << " longest search time: "
              << rmf_traffic::time::to_seconds(_max) << std::endl;
  }
}

//==============================================================================
template<typename T>
std::optional<ConstForestSolutionPtr>
BidirectionalForest<T>::_check_for_solution(
  WaypointId start, WaypointId finish) const
{
  SpinLock lock(_solutions_mutex);
  const auto s_it = _solutions.find(start);
  if (s_it != _solutions.end())
  {
    const auto& finish_map = s_it->second;
    const auto f_it = finish_map.find(finish);
    if (f_it != finish_map.end())
      return f_it->second;
  }

  return std::nullopt;
}

//==============================================================================
template<typename T>
ConstForestSolutionPtr BidirectionalForest<T>::_search(
  WaypointId start,
  std::optional<LockedTree<ForwardTree>> forward_locked,
  WaypointId finish,
  std::optional<LockedTree<ReverseTree>> reverse_locked) const
{
  std::optional<Timer<T>> timer;
  if constexpr (T::max_timer || T::print_timers)
  {
    timer = Timer<T>(
      std::string(typeid(T).name()) + ": " + std::to_string(start)
      + " -> " + std::to_string(finish), _max);
  }

  if constexpr (T::count_usage)
    ++_search_count;

  using GetKey = typename T::GetKey;

  ConstForestSolutionPtr result;
  std::vector<ForwardNodePtr> new_forwards;
  std::vector<ReverseNodePtr> new_reverses;
  {
    auto& forward = *forward_locked->tree;
    forward.retarget(_heuristic_cache, finish);

    auto& reverse = *reverse_locked->tree;
    reverse.retarget(_heuristic_cache, start);

    while (!(forward.exhausted() && reverse.exhausted()))
    {
      {
        const auto next_forward = forward.expand();
        if (next_forward)
        {
          if (T::cross_polinate)
            new_forwards.push_back(next_forward);

          if (const auto overlap = reverse.visited(GetKey()(next_forward)))
          {
            result = std::make_shared<ForestSolution>(
              combine_paths(*next_forward, *overlap));
            break;
          }
        }
      }

      if (T::grow_bidirectional)
      {
        const auto next_reverse = reverse.expand();
        if (next_reverse)
        {
          if (T::cross_polinate)
            new_reverses.push_back(next_reverse);

          if (const auto overlap = forward.visited(GetKey()(next_reverse)))
          {
            result = std::make_shared<ForestSolution>(
              combine_paths(*next_reverse, *overlap));
            break;
          }
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
      auto& manager = *get_manager(_forward, GetKey()(node))->second;
      manager.add_to_waiting(node);
    }
  }

  {
    SpinLock lock(_reverse_mutex);
    for (const auto& node : new_forwards)
    {
      auto& manager = *get_manager(_reverse, GetKey()(node))->second;
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

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_TREE_HPP
