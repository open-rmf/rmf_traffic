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

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

using LaneId = std::size_t;
using WaypointId = std::size_t;

using ChildOfMinimalTravelHeuristic = ShortestPathHeuristic;
using ChildHeuristicFactory = ShortestPathHeuristicFactory;

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
    if (!a->remaining_cost_estimate.has_value())
      return true;

    if (!b->remaining_cost_estimate.has_value())
      return false;

    // We want the elements ordered left-to-right from greatest to least
    return *a->remaining_cost_estimate + a->current_cost
        > *b->remaining_cost_estimate + b->current_cost;
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

private:
  std::vector<Element> _storage;
  Compare _comparator;
};

//==============================================================================
template<typename NodeT>
class Expander
{
public:

  using Node = NodeT;
  using NodePtr = std::shared_ptr<Node>;
  using Frontier = FrontierTemplate<NodePtr, OptionalCompare<NodePtr>>;

  virtual NodePtr expand(
    const NodePtr& top,
    Frontier& frontier,
    std::unordered_map<LaneId, NodePtr>& visited) const = 0;

  virtual void initialize(
    std::size_t waypoint,
    Frontier& frontier) const = 0;

  virtual void retarget(
    Cache<ChildOfMinimalTravelHeuristic> new_heuristic,
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

  Tree(
    std::size_t initial_waypoint,
    Expander expander);

  NodePtr expand();

  void insert(NodePtr node);

  NodePtr visited(LaneId lane_index) const;

  const std::unordered_map<LaneId, NodePtr>& all_visits() const;

  bool exhausted() const;

  void retarget(Cache<ChildOfMinimalTravelHeuristic> new_heuristic);

private:
  Frontier _frontier;
  std::unordered_map<LaneId, NodePtr> _visited;
  Expander _expander;
};

//==============================================================================
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

//==============================================================================
class ForwardExpander : public Expander<ForwardNode>
{
public:

  ForwardExpander(
    std::shared_ptr<const Supergraph> graph,
    Cache<ChildOfMinimalTravelHeuristic> heuristic);

  ForwardNodePtr expand(
    const ForwardNodePtr& top,
    Frontier& frontier,
    std::unordered_map<LaneId, ForwardNodePtr>& visited) const final;

  void initialize(
    std::size_t waypoint,
    Frontier& frontier) const final;

  void retarget(
    Cache<ChildOfMinimalTravelHeuristic> new_heuristic,
    Frontier &frontier) final;

private:
  std::shared_ptr<const Supergraph> _graph;
  Cache<ChildOfMinimalTravelHeuristic> _heuristic;
};

using ForwardTree = Tree<ForwardExpander>;

//==============================================================================
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

//==============================================================================
class ReverseExpander : public Expander<ReverseNode>
{
public:

  ReverseExpander(
    std::shared_ptr<const Supergraph> graph,
    Cache<ChildOfMinimalTravelHeuristic> heuristic);

  ReverseNodePtr expand(
    const ReverseNodePtr& top,
    Frontier& frontier,
    std::unordered_map<LaneId, ReverseNodePtr>& visited) const final;

  void initialize(
    std::size_t waypoint,
    Frontier& frontier) const final;

  void retarget(
    Cache<ChildOfMinimalTravelHeuristic> new_heuristic, Frontier& frontier) final;

private:
  std::shared_ptr<const Supergraph> _graph;
  Cache<ChildOfMinimalTravelHeuristic> _heuristic;
};

using ReverseTree = Tree<ReverseExpander>;

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

  using ComplementTree = ComplementTreeT;
  using ComplementNode = typename ComplementTree::Node;
  using ComplementNodePtr = typename ComplementTree::NodePtr;

  LockedTree<Tree> get_tree(
    std::size_t waypoint,
    const std::shared_ptr<const Supergraph>& graph,
    const Cache<ChildOfMinimalTravelHeuristic>& heuristic);

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

//==============================================================================
using ForwardTreeManager = TreeManager<ForwardTree, ReverseTree>;
using ReverseTreeManager = TreeManager<ReverseTree, ForwardTree>;

template<typename T, typename C>
using TreeManagerMap =
  std::unordered_map<WaypointId, std::unique_ptr<TreeManager<T, C>>>;
using ForwardTreeManagerMap = TreeManagerMap<ForwardTree, ReverseTree>;
using ReverseTreeManagerMap = TreeManagerMap<ReverseTree, ForwardTree>;

//==============================================================================
class MinimalTravelHeuristic
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
  CacheManagerMap<ChildHeuristicFactory> _heuristic_cache;

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

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP
