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

#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_MINIMALTRAVELHEURISTIC_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_MINIMALTRAVELHEURISTIC_HPP

#include "MinimalTravelHeuristic.hpp"

#include <rmf_utils/math.hpp>

namespace rmf_traffic {
namespace agv {
namespace planning {

namespace {
//==============================================================================
template<
  std::size_t Traversal::* get_next_lane,
  std::size_t Traversal::* get_next_waypoint,
  std::size_t Traversal::* get_complement_waypoint,
  typename NodePtrT, typename C>
void expand_traversals(
  const NodePtrT& top,
  FrontierTemplate<NodePtrT, C>& frontier,
  std::unordered_map<LaneId, NodePtrT>& visited,
  const std::shared_ptr<const Supergraph>& graph,
  const std::function<std::optional<double>(WaypointId)>& heuristic,
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
        for (const auto& alternative : traversal.alternatives)
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
        rotational_cost = time::to_seconds(
          internal::estimate_rotation_time(
            traits.rotational().get_nominal_velocity(),
            traits.rotational().get_nominal_acceleration(),
            0.0, *minimum_angle,
            graph->options().rotation_thresh));
      }
    }

    const auto next_waypoint = traversal.*get_next_waypoint;
    frontier.push(
      std::make_shared<typename NodePtrT::element_type>(
        typename NodePtrT::element_type{
          next_lane,
          top->current_cost + traversal.best_cost + rotational_cost,
          heuristic(next_waypoint),
          traversal.best_cost,
          next_waypoint,
          traversal.*get_complement_waypoint,
          next_orientation,
          top
        }));
  }
}

//==============================================================================
template<
  std::size_t Traversal::* get_next_lane,
  std::size_t Traversal::* get_next_waypoint,
  std::size_t Traversal::* get_complement_waypoint,
  typename NodePtrT, typename C>
void initialize_traversals(
  FrontierTemplate<NodePtrT, C>& frontier,
  const std::function<std::optional<double>(WaypointId)>& heuristic,
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

    const auto next_waypoint = traversal.*get_next_waypoint;
    frontier.push(
      std::make_shared<typename NodePtrT::element_type>(
        typename NodePtrT::element_type{
          traversal.*get_next_lane,
          traversal.best_cost,
          heuristic(next_waypoint),
          traversal.best_cost,
          next_waypoint,
          traversal.*get_complement_waypoint,
          orientation,
          nullptr
        }));
  }
}
} // anonymous namespace

} // namespace rmf_traffic
} // namespace agv
} // namespace planning

#endif // SRC__RMF_TRAFFIC__AGV__PLANNING__IMPL_MINIMALTRAVELHEURISTIC_HPP
