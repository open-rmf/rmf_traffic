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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP

#include "CacheManager.hpp"
#include "Supergraph.hpp"

#include "EuclideanHeuristic.hpp"
#include "Tree.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
/// The ShortestPathHeuristic finds the shortest (in terms of time required)
/// path between two waypoints, not accounting for acceleration, deceleration,
/// turning, or orientation constraints.
class ShortestPath : public DefaultForestSettings
{
public:

  using HeuristicCachePtr =
    std::shared_ptr<const CacheManagerMap<EuclideanHeuristicFactory>>;

  template<typename F>
  static bool exhausted(const F& frontier)
  {
    return frontier.empty();
  }

  struct ForwardNode;
  using ForwardNodePtr = std::shared_ptr<ForwardNode>;
  struct ForwardNode
  {
    std::size_t waypoint;
    double current_cost;
    ForwardNodePtr parent;
  };

  class ForwardExpander
    : public Expander<ForwardNode, HeuristicCachePtr, DijkstraCompare>
  {
  public:

    ForwardExpander(
      std::shared_ptr<const Supergraph> graph,
      const HeuristicCachePtr& cache,
      const WaypointId target);

    ForwardNodePtr expand(
      const ForwardNodePtr& top,
      Frontier& frontier,
      std::unordered_map<WaypointId, ForwardNodePtr>& visited) const final;

    void initialize(std::size_t waypoint, Frontier& frontier) const final;

    void retarget(
      const Cache& cache,
      WaypointId new_target,
      Frontier& frontier) final;

    bool exhausted(const Frontier& frontier) const final;

  private:
    std::shared_ptr<const Supergraph> _graph;
    double _max_speed;
  };
  using ForwardTree = Tree<ForwardExpander>;


  struct ReverseNode;
  using ReverseNodePtr = std::shared_ptr<ReverseNode>;
  struct ReverseNode
  {
    std::size_t waypoint;
    double current_cost;
    ReverseNodePtr parent;
  };

  struct GetKey
  {
    WaypointId operator()(const ForwardNodePtr& node) const
    {
      return node->waypoint;
    }

    WaypointId operator()(const ReverseNodePtr& node) const
    {
      return node->waypoint;
    }
  };

  class ReverseExpander
    : public Expander<ReverseNode, HeuristicCachePtr, DijkstraCompare>
  {
  public:

    ReverseExpander(
      std::shared_ptr<const Supergraph> graph,
      const HeuristicCachePtr& cache,
      const WaypointId target);

    ReverseNodePtr expand(
      const ReverseNodePtr& top,
      Frontier& frontier,
      std::unordered_map<WaypointId, ReverseNodePtr>& visited) const final;

    void initialize(std::size_t waypoint, Frontier& frontier) const final;

    void retarget(
      const Cache& cache,
      WaypointId new_target,
      Frontier& frontier) final;

    bool exhausted(const Frontier& frontier) const final;

  private:
    std::shared_ptr<const Supergraph> _graph;
    double _max_speed;
  };

  using ReverseTree = Tree<ReverseExpander>;

  using ForwardTreeManager = TreeManager<ForwardTree, ReverseTree>;
  using ReverseTreeManager = TreeManager<ReverseTree, ForwardTree>;

  using ForwardTreeManagerMap = TreeManagerMap<ForwardTree, ReverseTree>;
  using ReverseTreeManagerMap = TreeManagerMap<ReverseTree, ForwardTree>;

  template<typename T, typename C>
  static std::vector<typename T::NodePtr> flip_node(
    C current_node,
    const T& tree)
  {
    std::vector<typename T::NodePtr> new_nodes;
    const double full_cost = current_node->current_cost;
    typename T::NodePtr parent_node = nullptr;
    while (current_node)
    {
      if (const auto existing_node = tree.visited(current_node->waypoint))
      {
        parent_node = existing_node;
      }
      else
      {
        auto new_node = std::make_shared<typename T::Node>(
          typename T::Node{
            current_node->waypoint,
            full_cost - current_node->current_cost,
            parent_node
          });

        parent_node = new_node;
        new_nodes.emplace_back(std::move(new_node));
      }

      current_node = current_node->parent;
    }

    return new_nodes;
  }
};

//==============================================================================
class ShortestPathHeuristic : public BidirectionalForest<ShortestPath>
{
public:

  ShortestPathHeuristic(
    std::shared_ptr<const Supergraph> graph);

};

//==============================================================================
inline ForestSolution combine_paths(
  const ShortestPath::ForwardNode& a,
  const ShortestPath::ReverseNode& b)
{
  const double cost = a.current_cost + b.current_cost;
  std::vector<std::size_t> path;
  path.push_back(a.waypoint);
  auto f_node = a.parent;
  while (f_node)
  {
    path.push_back(f_node->waypoint);
    f_node = f_node->parent;
  }

  // Crawling down the forward path gives us a backtrack of the path, so we need
  // to reverse it here.
  std::reverse(path.begin(), path.end());

  auto r_node = b.parent;
  while (r_node)
  {
    path.push_back(r_node->waypoint);
    r_node = r_node->parent;
  }

  return ForestSolution{cost, std::move(path)};
}

//==============================================================================
inline ForestSolution combine_paths(
  const ShortestPath::ReverseNode& b,
  const ShortestPath::ForwardNode& a)
{
  return combine_paths(a, b);
}

//==============================================================================
inline std::vector<ShortestPath::ReverseNodePtr> flip_node(
  ShortestPath::ForwardNodePtr current_node,
  const ShortestPath::ReverseTree& tree)
{
  return ShortestPath::flip_node(current_node, tree);
}

//==============================================================================
inline std::vector<ShortestPath::ForwardNodePtr> flip_node(
  ShortestPath::ReverseNodePtr current_node,
  const ShortestPath::ForwardTree& tree)
{
  return ShortestPath::flip_node(current_node, tree);
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__SHORTESTPATHHEURISTIC_HPP
