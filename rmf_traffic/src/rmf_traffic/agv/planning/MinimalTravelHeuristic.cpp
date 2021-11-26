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
          traversal.best_time + rotational_cost,
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
std::optional<LaneId> ForwardExpander::expand(
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
    return std::nullopt;
  }

  const auto& top_lane = _graph->original().lanes.at(top->lane);
  const auto waypoint = top_lane.exit().waypoint_index();
  expand_traversals<&Traversal::finish_lane_index>(
    top,
    frontier,
    visited,
    _graph,
    *_graph->traversals_from(waypoint));

  return top->lane;
}

//==============================================================================
void ForwardExpander::initialize(
  std::size_t waypoint_index,
  DijkstraQueue<NodePtrT>& frontier) const
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
std::optional<LaneId> ReverseExpander::expand(
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
    return std::nullopt;
  }

  const auto& top_lane = _graph->original().lanes.at(top->lane);
  const auto waypoint = top_lane.entry().waypoint_index();
  expand_traversals<&Traversal::initial_lane_index>(
    top,
    frontier,
    visited,
    _graph,
    *_graph->traversals_into(waypoint));

  return top->lane;
}

//==============================================================================
void ReverseExpander::initialize(
  std::size_t waypoint_index,
  DijkstraQueue<NodePtrT>& frontier) const
{
  const auto& traversals = *_graph->traversals_into(waypoint_index);
  initialize_traversals<&Traversal::initial_lane_index>(frontier, traversals);
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
