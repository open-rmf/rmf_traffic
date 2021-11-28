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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__TREE_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__TREE_HPP

#include <algorithm>
#include <memory>
#include <unordered_map>
#include "CacheManager.hpp"
#include "Supergraph.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

using LaneId = std::size_t;
using WaypointId = std::size_t;

////==============================================================================
//template<typename NodePtrT>
//struct DijkstraCompare
//{
//  bool operator()(const NodePtrT& a, const NodePtrT& b)
//  {
//    // Note(MXG): The priority queue puts the greater value first, so we
//    // reverse the arguments in this comparison.
//    return b->cost < a->cost;
//  }
//};

////==============================================================================
//template<typename NodePtrT>
//using DijkstraQueue = std::priority_queue<
//  NodePtrT, std::vector<NodePtrT>, DijkstraCompare<NodePtrT>>;

template<typename NodePtrT>
struct OptionalCompare
{
  bool operator()(const NodePtrT& a, const NodePtrT& b)
  {
//    if (!a->remaining_cost_estimate.has_value())
//    {
//      if (!b->remaining_cost_estimate.has_value())
//        return b->current_cost < a->current_cost;

//      return false;
//    }

//    if (!b->remaining_cost_estimate.has_value())
//      return false;

    constexpr auto inf = std::numeric_limits<double>::infinity();
    return b->remaining_cost_estimate.value_or(inf) + b->current_cost
      < a->remaining_cost_estimate.value_or(inf) + a->current_cost;
  }
};

//==============================================================================
template<typename ElementT, typename CompareT>
class FrontierTemplate
{
public:

  using Element = ElementT;
  using Compare = CompareT;

  FrontierTemplate(Compare comparator = Compare());

  Element pop();

  const Element* peek() const;

  void push(Element new_element);

  bool empty() const;

  void retarget(std::function<void(Element&)> transform);

  const std::vector<Element>& storage() const;

private:
  std::vector<Element> _storage;
  Compare _comparator;
};

//==============================================================================
template<typename NodeT, typename CacheT>
class Expander
{
public:

  using Node = NodeT;
  using NodePtr = std::shared_ptr<Node>;
  using Frontier = FrontierTemplate<NodePtr, OptionalCompare<NodePtr>>;
  using Cache = CacheT;

  virtual NodePtr expand(
    const NodePtr& top,
    Frontier& frontier,
    std::unordered_map<std::size_t, NodePtr>& visited) const = 0;

  virtual void initialize(
    std::size_t waypoint,
    Frontier& frontier) const = 0;

  virtual void retarget(
    const Cache& cache,
    WaypointId new_target,
    Frontier& frontier) = 0;
};

//==============================================================================
template<typename ExpanderT>
class Tree
{
public:

  using Expander = ExpanderT;
  using Node = typename Expander::Node;
  using NodePtr = typename Expander::NodePtr;
  using Frontier = typename Expander::Frontier;
  using Cache = typename Expander::Cache;

  Tree(
    std::size_t initial_waypoint,
    Expander expander);

  NodePtr expand();

  void insert(NodePtr node);

  NodePtr visited(LaneId lane_index) const;

  const std::unordered_map<LaneId, NodePtr>& all_visits() const;

  bool exhausted() const;

  void retarget(
    const Cache& cache,
    WaypointId new_target);

  const Frontier& frontier() const;

private:
  Frontier _frontier;
  std::unordered_map<LaneId, NodePtr> _visited;
  Expander _expander;
};

//==============================================================================
template<typename TreeT>
struct LockedTree
{
  using Tree = TreeT;

  Tree* tree;
  SpinLock lock;
};

//==============================================================================
template<typename TreeT, typename ComplementTreeT>
class TreeManager
{
public:

  using Tree = TreeT;
  using Node = typename Tree::Node;
  using NodePtr = typename Tree::NodePtr;
  using Cache = typename Tree::Cache;

  using ComplementTree = ComplementTreeT;
  using ComplementNode = typename ComplementTree::Node;
  using ComplementNodePtr = typename ComplementTree::NodePtr;

  LockedTree<Tree> get_tree(
    std::size_t root_waypoint,
    const std::shared_ptr<const Supergraph>& graph,
    const Cache& heuristic,
    std::size_t target_waypoint);

  void add_to_waiting(ComplementNodePtr node);

private:

  void _process_waiting_list();

  std::optional<Tree> _tree;
  std::atomic_bool _tree_mutex = false;

  // This field contains the solution node of the minimum cost path from some
  // other waypoint to the one for this tree's root. These nodes should be
  // flipped and inserted into the tree at the start of any usage of this tree.
  std::vector<ComplementNodePtr> _waiting_list;
  std::atomic_bool _waiting_list_mutex = false;
};

template<typename T, typename C>
using TreeManagerMap =
  std::unordered_map<WaypointId, std::unique_ptr<TreeManager<T, C>>>;

//==============================================================================
template<typename T>
class Garden
{
public:

  using ForwardTree = typename T::ForwardTree;
  using ForwardNode = typename ForwardTree::Node;
  using ForwardNodePtr = typename ForwardTree::NodePtr;

  using ReverseTree = typename T::ReverseTree;
  using ReverseNode = typename ReverseTree::Node;
  using ReverseNodePtr = typename ReverseTree::NodePtr;

  using Cache = typename ForwardTree::Cache;

  Garden(
    std::shared_ptr<const Supergraph> graph,
    Cache cache);

  std::optional<double> get(WaypointId start, WaypointId finish) const;

private:

  using ForwardTreeManagerMap = TreeManagerMap<ForwardTree, ReverseTree>;
  using ReverseTreeManagerMap = TreeManagerMap<ReverseTree, ForwardTree>;

  std::optional<double> _check_for_solution(
    WaypointId start, WaypointId finish) const;

  std::optional<double> _search(
    WaypointId start,
    std::optional<LockedTree<ForwardTree>> forward_locked,
    WaypointId finish,
    std::optional<LockedTree<ReverseTree>> reverse_locked) const;

  std::shared_ptr<const Supergraph> _graph;
  Cache _heuristic_cache;

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

} // namespace rmf_traffic
} // namespace agv
} // namespace planning

#include "impl_Tree.hpp"

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__TREE_HPP
