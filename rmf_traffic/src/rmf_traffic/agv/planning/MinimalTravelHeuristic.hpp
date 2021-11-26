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

#include <queue>

namespace rmf_traffic {
namespace agv {
namespace planning {

using LaneId = std::size_t;

//==============================================================================
template<typename NodePtrT>
struct DijkstraCompare
{
  bool operator()(const NodePtrT& a, const NodePtrT& b)
  {
    // Note(MXG): The priority queue puts the greater value first, so we
    // reverse the arguments in this comparison.
    return b->cost < a->cost;
  }
};

//==============================================================================
template<typename NodePtrT>
using DijkstraQueue = std::priority_queue<
  NodePtrT, std::vector<NodePtrT>, DijkstraCompare<NodePtrT>>;

//==============================================================================
template<typename NodeT>
class Expander
{
public:

  using NodePtrT = std::shared_ptr<const NodeT>;

  virtual std::optional<LaneId> expand(
    const NodePtrT& top,
    DijkstraQueue<NodePtrT>& frontier,
    std::unordered_map<LaneId, NodePtrT>& visited) const = 0;

  virtual void initialize(
    std::size_t waypoint,
    DijkstraQueue<NodePtrT>& frontier) const = 0;

};

//==============================================================================
template<typename Expander>
class Tree
{
public:

  using NodeT = typename Expander::NodeT;
  using NodePtrT = std::shared_ptr<const NodeT>;

  Tree(
    std::size_t initial_waypoint,
    Expander expander);

  std::optional<LaneId> expand();

  void insert(NodePtrT node);

  bool exhausted() const;

private:
  DijkstraQueue<NodePtrT> _frontier;
  std::unordered_map<LaneId, NodePtrT> _visited;
  Expander _expander;
};

//==============================================================================
struct ForwardNode;
using ForwardNodePtr = std::shared_ptr<const ForwardNode>;
struct ForwardNode
{
  LaneId lane;
  double cost;
  std::optional<double> orientation;
  ForwardNodePtr parent;
};

//==============================================================================
class ForwardExpander : public Expander<ForwardNode>
{
public:

  ForwardExpander(std::shared_ptr<const Supergraph> graph);

  std::optional<LaneId> expand(
    const ForwardNodePtr& top,
    DijkstraQueue<ForwardNodePtr>& frontier,
    std::unordered_map<LaneId, ForwardNodePtr>& visited) const final;

  void initialize(
    std::size_t waypoint,
    DijkstraQueue<ForwardNodePtr>& frontier) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
};

using ForwardTree = Tree<ForwardExpander>;

//==============================================================================
struct ReverseNode;
using ReverseNodePtr = std::shared_ptr<const ReverseNode>;
struct ReverseNode
{
  LaneId lane;
  double cost;
  std::optional<double> orientation;
  ReverseNodePtr parent;
};

//==============================================================================
class ReverseExpander : public Expander<ReverseNode>
{
public:

  ReverseExpander(std::shared_ptr<const Supergraph> graph);

  std::optional<LaneId> expand(
    const ReverseNodePtr& top,
    DijkstraQueue<ReverseNodePtr>& frontier,
    std::unordered_map<LaneId, ReverseNodePtr>& visited) const final;

  void initialize(
    std::size_t waypoint,
    DijkstraQueue<ReverseNodePtr>& frontier) const final;

private:
  std::shared_ptr<const Supergraph> _graph;
};

using ReverseTree = Tree<ReverseExpander>;

//==============================================================================
class MinimalTravelHeuristic
{
public:

  MinimalTravelHeuristic(
    std::shared_ptr<const Supergraph> graph);

private:
  std::shared_ptr<const Supergraph> _graph;

};

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__MINIMALTRAVELHEURISTIC_HPP
