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

#include "DifferentialDrivePlanner.hpp"

#include "../internal_Planner.hpp"

#include "a_star.hpp"

#include <rmf_utils/math.hpp>
#include <set>

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
#include <iostream>
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

namespace rmf_traffic {
namespace agv {
namespace planning {

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(const NodePtr& finish_node)
{
  NodePtr node = finish_node;
  std::vector<NodePtr> node_sequence;
  while (node)
  {
    node_sequence.push_back(node);
    node = node->parent;
  }

  std::reverse(node_sequence.begin(), node_sequence.end());

  return node_sequence;
}

namespace {
//==============================================================================
template<typename NodePtr>
void reparent_node_for_holding(
  const NodePtr& low_node,
  const NodePtr& high_node,
  Route route)
{
  // TODO(MXG): Rework this implementation so that the pointed-to types can
  // be const-qualified.
  high_node->parent = low_node;
  high_node->route_from_parent = {std::move(route)};

  // Clear any approach lanes that the parent had, because now the robot will
  // simply be sitting in place.
  high_node->approach_lanes.clear();
}

//==============================================================================
template<typename NodePtr>
struct OrientationTimeMap
{
  struct Element
  {
    using TimeMap = std::map<rmf_traffic::Time, NodePtr>;
    using TimePair = std::pair<
      typename TimeMap::iterator,
      typename TimeMap::iterator
    >;

    double yaw;
    TimeMap time_map;

    void squash(const agv::RouteValidator* validator)
    {
      assert(!time_map.empty());
      if (time_map.size() <= 2)
        return;

      std::vector<TimePair> queue;
      queue.push_back({time_map.begin(), --time_map.end()});

      while (!queue.empty())
      {
        const auto top = queue.back();
        queue.pop_back();
        const auto it_low = top.first;
        const auto it_high = top.second;

        assert(it_low != time_map.end());
        assert(it_high != time_map.end());

        if (it_high->first <= it_low->first)
          continue;

        if (it_low == --typename TimeMap::iterator(it_high))
        {
          // If the iterators are perfectly next to each other, then the only
          // action between them must be a holding.
          continue;
        }

        assert(it_high != time_map.begin());

        const auto& node_low = it_low->second;
        const auto& node_high = it_high->second;

        const auto& start_wp =
          node_low->route_from_parent.back().trajectory().back();

        const auto& end_wp =
          node_high->route_from_parent.back().trajectory().back();

        Route new_route{node_low->route_from_parent.back().map(), {}};
        new_route.trajectory().insert(start_wp);
        new_route.trajectory().insert(
          end_wp.time(),
          end_wp.position(),
          Eigen::Vector3d::Zero());

        if (!validator || !validator->find_conflict(new_route))
        {
          reparent_node_for_holding(node_low, node_high, std::move(new_route));
          time_map.erase(++typename TimeMap::iterator(it_low), it_high);
          queue.clear();
          if (time_map.size() > 2)
            queue.push_back({time_map.begin(), --time_map.end()});

          continue;
        }

        const auto next_high = --typename TimeMap::iterator(it_high);
        const auto next_low = ++typename TimeMap::iterator(it_low);

        if (next_high != it_low)
          queue.push_back({it_low, next_high});

        if (next_low != it_high)
          queue.push_back({next_low, it_high});
      }
    }
  };

  void insert(NodePtr node)
  {
    const auto yaw = node->yaw;
    const auto time =
      *node->route_from_parent.back().trajectory().finish_time();

    auto it = elements.begin();
    for (; it != elements.end(); ++it)
    {
      // TODO(MXG): Make this condition configurable
      if (std::abs(it->yaw - yaw) < 15.0*M_PI/180.0)
        break;
    }

    if (it == elements.end())
      elements.push_back({yaw, {{time, node}}});
    else
      it->time_map.insert({time, node});
  }

  std::vector<Element> elements;
};
} // anonymous namespace

//==============================================================================
template<typename NodePtr>
std::vector<NodePtr> reconstruct_nodes(
  const NodePtr& finish_node,
  const agv::RouteValidator* validator,
  const double w_nom,
  const double alpha_nom,
  const double rotational_threshold)
{
  auto node_sequence = reconstruct_nodes(finish_node);
  std::optional<rmf_traffic::agv::Plan::Start> start;
  if (!node_sequence.empty())
  {
    start = node_sequence.front()->start;
    if (!start.has_value())
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[rmf_traffic::agv::Planner::plan][1] The root node of a solved plan "
        "is missing its Start information. This should not happen. Please "
        "report this critical bug to the maintainers of rmf_traffic.");
      // *INDENT-ON*
    }
  }

  // Remove "cruft" from plans. This means making sure vehicles don't do any
  // unnecessary motions.
  std::unordered_map<
    std::size_t,
    OrientationTimeMap<NodePtr>
  > cruft_map;

  NodePtr first_midlane_node;
  NodePtr last_midlane_node;

  for (const auto& node : node_sequence)
  {
    if (!node->waypoint)
    {
      if (!first_midlane_node)
        first_midlane_node = node;
      else
        last_midlane_node = node;
    }
    else
    {
      const auto wp = *node->waypoint;
      cruft_map[wp].insert(node);
    }
  }

  if (first_midlane_node && last_midlane_node)
  {
    // There are multiple midlane nodes. We can ensure they are reduced to just
    // two by squashing them here. We do not have to check this against the
    // validator because the only way a robot is allowed to have multiple
    // midlane nodes is if it was able to sit in place.
    last_midlane_node->parent = first_midlane_node;

    rmf_traffic::Trajectory holding;
    const Eigen::Vector2d x = first_midlane_node->position;
    Eigen::Vector3d p0{x[0], x[1], first_midlane_node->yaw};
    Eigen::Vector3d p1{x[0], x[1], last_midlane_node->yaw};
    internal::interpolate_rotation(
        holding, w_nom, alpha_nom, first_midlane_node->time,
        p0, p1, rotational_threshold);
  }

  for (auto& cruft : cruft_map)
  {
    for (auto& duplicate : cruft.second.elements)
      duplicate.squash(validator);
  }

  auto final_node_sequence = reconstruct_nodes(finish_node);
  if (!final_node_sequence.empty())
  {
    final_node_sequence.front()->start = start;
  }
  return final_node_sequence;
}

//==============================================================================
struct WaypointCandidate
{
  bool necessary;
  Plan::Waypoint::Implementation waypoint;
  Eigen::Vector3d velocity;
};

//==============================================================================
void stream_trajectory(
  std::ostream& ss,
  const Trajectory& traj)
{
  for (const auto& wp : traj)
  {
    ss << wp.index() << ". t=" << time::to_seconds(wp.time().time_since_epoch())
              << " p=(" << wp.position().transpose()
              << ") v=<" << wp.velocity().transpose() << "> --> ";
  }
  ss << "(finished)\n";
}

//==============================================================================
inline bool same_orientation(double yaw0, double yaw1)
{
  return std::abs(rmf_utils::wrap_to_pi(yaw0 - yaw1))*180.0 / M_PI < 1e-2;
}

//==============================================================================
std::vector<Plan::Waypoint> find_dependencies(
  std::vector<Route>& itinerary,
  std::vector<WaypointCandidate> candidates,
  const RouteValidator* validator,
  const std::optional<Duration> dependency_window,
  const Duration dependency_resolution)
{
  using PlanWaypointId = std::size_t;
  std::vector<std::map<CheckpointId, PlanWaypointId>> checkpoint_maps;
  checkpoint_maps.resize(itinerary.size());

  for (std::size_t i = 0; i < candidates.size(); ++i)
  {
    for (const auto& c : candidates[i].waypoint.arrival)
    {
      // There may be duplicate insertions because of event waypoints, but
      // that's okay. We just use the first relevant plan waypoint and allow the
      // insertion to fail for the rest.
      checkpoint_maps.at(c.route_id).insert({c.checkpoint_id, i});
    }
  }


  if (validator && dependency_window.has_value())
  {
    for (std::size_t i = 0; i < itinerary.size(); ++i)
    {
      auto& route = itinerary[i];
      auto& checkpoint_map = checkpoint_maps[i];
      if (checkpoint_map.empty())
        continue;

      assert(route.trajectory().start_time());
      const auto initial_time = *route.trajectory().start_time();

      bool no_conflicts = false;
      bool anomaly_happened = false;
      std::size_t count = 0;
      std::unordered_map<CheckpointId, Dependencies> found_dependencies;
      while (!no_conflicts)
      {
        if (++count > 100)
        {
          // This almost certainly means there's a bug causing an infinite loop.
          // A normal value would be less than 10.
          throw std::runtime_error(
            "[rmf_traffic::agv::Planner::plan] Excessive iterating while "
            "searching for plan dependencies. This likely indicates a bug in "
            "the RouteValidator that was provided.");
        }

        no_conflicts = true;

        for (auto t = dependency_resolution;
             t < *dependency_window; t += dependency_resolution)
        {
          route.trajectory().front().adjust_times(-dependency_resolution);
          const auto conflict = validator->find_conflict(route);
          if (conflict.has_value())
          {
            auto it = route.trajectory().lower_bound(conflict->time);
            if (it != route.trajectory().begin())
            {
              // NOTE: If a conflict is detected with the exact start of the
              // trajectory then we ignore it because it is not physically
              // meaningful and there isn't anything we could do about it
              // anyway.
              --it;
              const auto dependent = it->index();
              const Dependency dependency = conflict->dependency;

              auto& found_deps = found_dependencies[dependent];
              const auto f_it = std::find(
                found_deps.begin(), found_deps.end(), dependency);

              if (f_it != found_deps.end())
              {
                std::stringstream ss;
                ss << "-------------------------------------------------"
                   << "\n[rmf_traffic::agv::Planner::plan] WARNING: "
                   << "A rare anomaly has occurred in the planner. The Route "
                   << "Validator has failed to recognize a specified route "
                   << "dependency: " << dependent << " on {"
                   << dependency.on_participant << " " << dependency.on_plan
                   << " " << dependency.on_route << " "
                   << dependency.on_checkpoint << "}. These are the "
                   << "trajectories:\n";
                stream_trajectory(ss, route.trajectory());
                ss << "vs\n";
                stream_trajectory(ss, conflict->route->trajectory());
                ss << "Please provide this information to the RMF developers "
                   << "for debugging.\n"
                   << "-------------------------------------------------\n";
                std::cerr << ss.str() << std::endl;
                anomaly_happened = true;
                break;
              }

              no_conflicts = false;
              found_deps.push_back(dependency);
              const auto wp = [&]() -> std::optional<CheckpointId>
              {
                assert(!checkpoint_map.empty());
                // Find the closest route waypoint less than or equal to
                // `dependent` that is associated with a plan waypoint.
                auto c_it = checkpoint_map.upper_bound(dependent);
                if (c_it == checkpoint_map.begin())
                {
                  // If the upper bound is the first element in the checkpoint
                  // map, then the real dependent is the previous route of this
                  // plan. We don't have a way to express that in today's RMF,
                  // so instead we will set the first checkpoint of this route
                  // as a dependent and then skip further dependency checking.
                  return std::nullopt;
                }

                --c_it;
                route.add_dependency(c_it->first, dependency);

                return c_it->second;
              } ();

              if (wp.has_value())
              {
                candidates[*wp].waypoint.dependencies.push_back(dependency);
                candidates[*wp].necessary = true;
              }
              else
              {
                candidates.front().waypoint.dependencies.push_back(dependency);
                candidates.front().necessary = true;
                anomaly_happened = true;
                break;
              }
            }
          }
        }

        if (anomaly_happened)
          break;

        const auto delta_t = initial_time - route.trajectory().front().time();
        route.trajectory().front().adjust_times(delta_t);
      }
    }
  }

  bool merge_happened = true;
  while (merge_happened)
  {
    merge_happened = false;
    std::optional<std::size_t> unnecessary_index_start;
    std::vector<Plan::Progress> progress;
    std::vector<std::size_t> approach_lanes;
    std::size_t i = 0;
    for (; i < candidates.size(); ++i)
    {
      if (!candidates[i].necessary)
      {
        if (!unnecessary_index_start.has_value())
          unnecessary_index_start = i;

        for (const auto& approach : candidates[i].waypoint.approach_lanes)
          approach_lanes.push_back(approach);

        progress.push_back(
          Plan::Progress{
            candidates[i].waypoint.graph_index.value(),
            candidates[i].waypoint.arrival,
            candidates[i].waypoint.time
          });
      }
      else if (unnecessary_index_start.has_value())
      {
        break;
      }
    }

    if (unnecessary_index_start.has_value())
    {
      merge_happened = true;
      assert(candidates[i].waypoint.progress.empty());
      candidates[i].waypoint.progress = std::move(progress);
      candidates[i].waypoint.approach_lanes.insert(
        candidates[i].waypoint.approach_lanes.begin(),
        approach_lanes.begin(), approach_lanes.end());

      candidates.erase(
        candidates.begin() + *unnecessary_index_start,
        candidates.begin() + i);
    }
  }

  std::size_t c_last = 0;
  std::size_t c = 1;
  std::size_t c_next = 2;
  for (; c_next < candidates.size(); ++c_last, ++c, ++c_next)
  {
    auto& candidate = candidates[c];
    if (candidate.waypoint.arrival.empty())
    {
      // This was a manually inserted turn-in-place, so let's try to match it
      // with a trajectory checkpoint
      const auto p_candidate = candidate.waypoint.position;
      for (std::size_t r = 0; r < itinerary.size(); ++r)
      {
        std::optional<std::size_t> floor_opt;
        for (const auto& checkpoint : candidates[c_last].waypoint.arrival)
        {
          if (checkpoint.route_id != r)
            continue;

          const auto id = checkpoint.checkpoint_id;
          if (!floor_opt.has_value())
            floor_opt = id;
          else if (*floor_opt < id)
            floor_opt = id;
        }

        if (!floor_opt.has_value())
          continue;
        const auto floor_id = floor_opt.value();

        std::optional<std::size_t> ceil_opt;
        for (const auto& checkpoint : candidates[c_next].waypoint.arrival)
        {
          if (checkpoint.route_id != r)
            continue;

          const auto id = checkpoint.checkpoint_id;
          if (!ceil_opt.has_value())
            ceil_opt = id;
          else if (id < *ceil_opt)
            ceil_opt = id;
        }

        if (!ceil_opt.has_value())
          continue;
        const auto ceil_id = ceil_opt.value();

        // Between the floor and the ceiling, try to find a waypoint in this
        // trajectory that matches the position of the inserted waypoint.
        for (std::size_t id = floor_id; id < ceil_id; ++id)
        {
          const auto& trajectory_checkpoint = itinerary[r].trajectory()[id];
          const auto p_trajectory = trajectory_checkpoint.position();
          const bool same_pos = (p_candidate.block<2,1>(0,0) - p_trajectory.block<2,1>(0,0)).norm() < 1e-2;
          const bool same_ori = same_orientation(p_candidate[2], p_trajectory[2]);
          if (same_pos && same_ori)
          {
            // We have a matching checkpoint
            candidate.waypoint.arrival.push_back(Plan::Checkpoint { r, id });
            candidate.waypoint.time = trajectory_checkpoint.time();
          }
        }
      }
    }
  }

  std::vector<Plan::Waypoint> waypoints;
  waypoints.reserve(candidates.size());
  for (const auto& c : candidates)
    waypoints.push_back(Plan::Waypoint::Implementation::make(c.waypoint));

  return waypoints;
}

//==============================================================================
template<typename NodePtr>
std::pair<std::vector<Route>, std::vector<Plan::Waypoint>>
reconstruct_waypoints(
  const std::vector<NodePtr>& node_sequence,
  const Graph::Implementation& graph,
  const RouteValidator* validator,
  const std::optional<Duration> dependency_window,
  const Duration dependency_resolution,
  const std::optional<rmf_traffic::Duration> span = std::nullopt)
{
  if (node_sequence.size() == 1)
  {
    std::vector<Route> output;
    // If there is only one node in the sequence, then it is a start node.
    if (span)
    {
      // When performing a rollout, it is important that at least one route with
      // two waypoints is provided. We use the span value to creating a
      // stationary trajectory when the robot is already starting out at a
      // holding point.

      // TODO(MXG): Make a unit test for this situation
      std::vector<Route> simple_route =
        node_sequence.back()->route_from_parent;
      if (simple_route.back().trajectory().size() < 2)
      {
        const auto& wp = simple_route.back().trajectory().back();
        for (auto&  r : simple_route)
        {
          r.trajectory().insert(
            wp.time() + *span,
            wp.position(),
            Eigen::Vector3d::Zero());
        }
      }

      output = std::move(simple_route);
    }

    // When there is only one node, we should return an empty itinerary to
    // indicate that the AGV does not need to go anywhere.
    return {output, {}};
  }

  const auto in_place_node_to_wp = [&](const NodePtr& n) -> WaypointCandidate
    {
      const Eigen::Vector2d p = n->waypoint ?
        graph.waypoints[*n->waypoint].get_location() :
        n->route_from_parent.back().trajectory().back().position()
        .template block<2, 1>(0, 0);
      return WaypointCandidate{
        true,
        Plan::Waypoint::Implementation{
          Eigen::Vector3d{p[0], p[1], n->yaw}, n->time, n->waypoint,
          n->approach_lanes, {}, {}, n->event, {}
        },
        Eigen::Vector3d{0, 0, 0}
      };
    };

  std::vector<WaypointCandidate> candidates;
  candidates.push_back(in_place_node_to_wp(node_sequence[0]));
  candidates.back().waypoint.arrival.push_back({0, 0});

  std::vector<Route> itinerary = node_sequence.front()->route_from_parent;
  for (std::size_t i = 1; i < node_sequence.size(); ++i)
  {
    const auto& node = node_sequence.at(i);
    const auto yaw = node->yaw;

    // Insert in-place rotation waypoint if the current and previous waypoints
    // are disconnected
    const auto& prev_candidate = candidates.back();
    const auto& prev_wp = prev_candidate.waypoint;
    const Eigen::Vector2d prev_pos =
      Eigen::Vector2d{prev_wp.position[0], prev_wp.position[1]};
    const bool same_pos = (prev_pos - node->position).norm() < 1e-3;
    const bool same_ori = same_orientation(prev_wp.position[2], node->yaw);
    if (!same_pos && !same_ori)
    {
      candidates.push_back(WaypointCandidate{
        true,
        Plan::Waypoint::Implementation{
          Eigen::Vector3d{prev_pos[0],  prev_pos[1], node->yaw},
          prev_wp.time, prev_wp.graph_index,
          {}, {}, {}, prev_wp.event, {}
        },
        prev_candidate.velocity
      });
    }

    if (node->approach_lanes.empty())
    {
      // This means we are doing an in-place rotation or a wait
      candidates.push_back(in_place_node_to_wp(node));
    }

    std::vector<std::size_t> skipped_lanes;
    for (std::size_t c = 0; c < node->approach_lanes.size(); ++c)
    {
      const auto lane_index = node->approach_lanes[c];
      const auto wp_index = graph.lanes[lane_index].exit().waypoint_index();
      const Graph::Waypoint& g_wp = graph.waypoints[wp_index];
      const auto& p = g_wp.get_location();
      const bool necessary = (c == node->approach_lanes.size()-1);
      const auto opt_tv = [&]() -> std::optional<TimeVelocity>
        {
          if (necessary)
            return TimeVelocity{node->time, {0, 0}};

          try
          {
            return interpolate_time_along_quadratic_straight_line(
              node->route_from_parent.back().trajectory(), p, 0.0);
          }
          catch(const std::runtime_error& e)
          {
            std::stringstream ss;
            ss << e.what()
               << "\nError triggered in [rmf_traffic::agv::Planner::plan]\n";
            if (node->route_from_parent.empty())
            {
              ss << "No trajectory in the node!";
            }
            else
            {
              ss << "Entire trajectory:";
              for (const rmf_traffic::Trajectory::Waypoint& wp : node->route_from_parent.back().trajectory())
              {
                ss << " t=" << time::to_seconds(wp.time().time_since_epoch())
                   << " p=(" << wp.position().transpose()
                   << ") v=<" << wp.velocity().transpose()
                   << "> -->";
              }
            }
            std::cerr << ss.str() << std::endl;

            return std::nullopt;
          }
        } ();
      if (!opt_tv.has_value())
      {
        std::stringstream ss;
        ss << "Failed to calculate the (time, velocity) of a midpoint moving "
           << "towards waypoint [";
        if (node->waypoint.has_value())
          ss << *node->waypoint;
        else
          ss << "undefined";
        ss << "]. Approach lanes include:";
        for (const auto& a_wp : node->approach_lanes)
          ss << " " << a_wp;
        ss << ". Failed on lane " << lane_index << ".";
        std::cerr << ss.str() << ss.str() << std::endl;
        skipped_lanes.push_back(lane_index);
        continue;
      }

      const auto [time, v] = *opt_tv;

      candidates.push_back({
        necessary,
        Plan::Waypoint::Implementation{
          Eigen::Vector3d{p[0], p[1], yaw}, time, wp_index,
          {lane_index}, {}, {}, necessary ? node->event : nullptr, {}
        },
        {v[0], v[1], 0.0}
      });
    }

    for (const auto& skipped : skipped_lanes)
    {
      if (candidates.empty())
      {
        std::cerr << "No candidates were produced for the node!" << std::endl;
      }
      else
      {
        candidates.back().waypoint.approach_lanes.push_back(skipped);
      }
    }

    for (const Route& next_route : node->route_from_parent)
    {
      Route* extended_route = nullptr;
      Route& last_route = itinerary.back();
      if (next_route.map() == last_route.map())
      {
        extended_route = &last_route;
        for (const auto& waypoint : next_route.trajectory())
        {
          last_route.trajectory().insert(waypoint);
        }
      }
      else
      {
        bool merged = false;
        if (last_route.trajectory().size() < 2)
        {
          // The last itinerary did not have enough waypoints, so we should
          // discard it.
          const std::size_t remove = itinerary.size() - 1;
          itinerary.pop_back();
          for (auto& candidate : candidates)
          {
            const auto r_it = std::remove_if(
              candidate.waypoint.arrival.begin(),
              candidate.waypoint.arrival.end(),
              [remove](const Plan::Checkpoint& c)
              {
                return c.route_id == remove;
              });
            candidate.waypoint.arrival.erase(
              r_it,
              candidate.waypoint.arrival.end());
          }

          if (!itinerary.empty())
          {
            Route& last_route = itinerary.back();
            if (next_route.map() == last_route.map())
            {
              merged = true;
              extended_route = &last_route;
              for (const auto& waypoint : next_route.trajectory())
              {
                last_route.trajectory().insert(waypoint);
              }
            }
          }
        }

        if (!merged)
        {
          itinerary.push_back(next_route);
          extended_route = &itinerary.back();
        }
      }

      if (!node->approach_lanes.empty())
      {
        for (std::size_t c = 0; c < node->approach_lanes.size() - 1; ++c)
        {
          const auto candidate_index =
            candidates.size() - node->approach_lanes.size() + c;
          WaypointCandidate& candidate = candidates[candidate_index];

          const auto index = extended_route->trajectory().insert(
            candidate.waypoint.time,
            candidate.waypoint.position,
            candidate.velocity).it->index();

          candidate.waypoint.arrival.push_back({itinerary.size()-1, index});
        }
      }

      candidates.back().waypoint.arrival
        .push_back({itinerary.size()-1, itinerary.back().trajectory().size()-1});
    }
  }

  std::vector<std::size_t> removals;
  for (std::size_t i=0; i < itinerary.size(); ++i)
  {
    if (itinerary[i].trajectory().size() < 2)
    {
      removals.push_back(i);
    }
  }

  for (auto r_it = removals.rbegin(); r_it != removals.rend(); ++r_it)
  {
    std::size_t remove = *r_it;
    itinerary.erase(itinerary.begin() + remove);
    for (WaypointCandidate& candidate : candidates)
    {
      auto c_it = candidate.waypoint.arrival.begin();
      while (c_it != candidate.waypoint.arrival.end())
      {
        Plan::Checkpoint& c = *c_it;
        if (c.route_id == remove)
        {
          candidate.waypoint.arrival.erase(c_it);
          continue;
        }

        if (c.route_id > remove)
        {
          --c.route_id;
        }

        ++c_it;
      }
    }
  }

  auto plan_waypoints = find_dependencies(
      itinerary, candidates, validator,
      dependency_window, dependency_resolution);

  return {itinerary, plan_waypoints};
}

//==============================================================================
template<typename NodePtr>
Plan::Start find_start(NodePtr node)
{
  while (node->parent)
    node = node->parent;

  if (!node->start.has_value())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_traffic::agv::Planner::plan][2] The root node of a processed plan "
      "is missing its Start information. This should not happen. Please report "
      "this critical bug to the maintainers of rmf_traffic.");
    // *INDENT-ON*
  }

  return node->start.value();
}

//==============================================================================
class ScheduledDifferentialDriveExpander
{
public:

  using Entry = DifferentialDriveMapTypes::Entry;
  using EntryHash = DifferentialDriveMapTypes::EntryHash;

  struct SearchNode;
  using SearchNodePtr = std::shared_ptr<SearchNode>;
  using ConstSearchNodePtr = std::shared_ptr<const SearchNode>;
  using NodePtr = SearchNodePtr;

  struct SearchNode
  {
    // We use optional here because start nodes don't always have an Entry value
    // or a waypoint. If this is a nullopt, then SearchNode::start should have a
    // value.
    std::optional<Entry> entry;
    std::optional<std::size_t> waypoint;
    std::vector<std::size_t> approach_lanes;
    Eigen::Vector2d position;
    double yaw;
    Time time;

    double remaining_cost_estimate;

    std::vector<Route> route_from_parent;

    // An event that should occur when this node is reached,
    // i.e. after route_from_parent has been traversed
    Graph::Lane::EventPtr event;

    double current_cost;
    std::optional<Planner::Start> start;
    SearchNodePtr parent;
    std::size_t line;

    double get_total_cost_estimate() const
    {
      return current_cost + remaining_cost_estimate;
    }

    double get_remaining_cost_estimate() const
    {
      return remaining_cost_estimate;
    }

    std::optional<Orientation> get_orientation() const
    {
      if (entry.has_value())
        return entry->orientation;

      return std::nullopt;
    }

    SearchNode(
      std::optional<Entry> entry_,
      std::optional<std::size_t> waypoint_,
      std::vector<std::size_t> approach_lanes_,
      Eigen::Vector2d position_,
      double yaw_,
      Time time_,
      double remaining_cost_estimate_,
      std::vector<Route> route_from_parent_,
      Graph::Lane::EventPtr event_,
      double current_cost_,
      std::optional<Planner::Start> start_,
      SearchNodePtr parent_,
      std::size_t line_)
    : entry(entry_),
      waypoint(waypoint_),
      approach_lanes(std::move(approach_lanes_)),
      position(position_),
      yaw(yaw_),
      time(time_),
      remaining_cost_estimate(remaining_cost_estimate_),
      route_from_parent(std::move(route_from_parent_)),
      event(event_),
      current_cost(current_cost_),
      start(std::move(start_)),
      parent(std::move(parent_)),
      line(line_)
    {
      assert(!route_from_parent.empty());

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (parent &&
        (parent->get_total_cost_estimate() > get_total_cost_estimate() + 1e-3))
      {
        std::cout << "Inadmissible expansion! "
                  << parent->current_cost << " + "
                  << parent->remaining_cost_estimate << " = "
                  << parent->get_total_cost_estimate()
                  << " --> "
                  << current_cost << " + "
                  << remaining_cost_estimate << " = "
                  << get_total_cost_estimate()
                  << " | Diff: "
                  << parent->get_total_cost_estimate() -
          get_total_cost_estimate()
                  << std::endl;

        std::cout << "Yaw: " << parent->yaw*180.0/M_PI << " --> "
                  << yaw*180.0/M_PI << " | Trans: ("
                  << parent->position.transpose() << ") --> ("
                  << position.transpose() << ") <"
                  << (parent->position - position).norm() << ">" << std::endl;
      }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (start.has_value())
      {
        std::cout << "making start node (" << route_from_parent.size()
                  << "):";
        for (const auto& r : route_from_parent)
          std::cout << " " << r.trajectory().size();
        std::cout << std::endl;

        std::cout << " -- [";
        if (waypoint)
          std::cout << waypoint.value();
        else
          std::cout << "nullopt";

        std::cout << ":" << start->waypoint() << "] -- <"
                  << position.transpose() << "; " << yaw << "> vs <";

        if (start->location().has_value())
          std::cout << start->location()->transpose();
        else
          std::cout << "nullopt";

        std::cout << "; " << start->orientation() << ">" << std::endl;
      }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
    }
  };

  using SearchQueue =
    std::priority_queue<
    SearchNodePtr,
    std::vector<SearchNodePtr>,
    DifferentialDriveCompare<SearchNodePtr>
    >;

  class InternalState : public State::Internal
  {
  public:

    std::optional<double> cost_estimate() const final
    {
      if (queue.empty())
        return std::nullopt;

      const auto& top = queue.top();
      return top->current_cost + top->remaining_cost_estimate;
    }

    std::size_t queue_size() const final
    {
      return queue.size();
    }

    std::size_t expansion_count() const final
    {
      return popped_count;
    }

    SearchQueue queue;
    std::size_t popped_count = 0;
  };

  bool quit(const SearchNodePtr& top, SearchQueue& queue) const
  {
    ++_internal->popped_count;

    if (_saturation_limit.has_value())
    {
      if (*_saturation_limit < _internal->popped_count + queue.size())
        return true;
    }

    if (_maximum_cost_estimate.has_value())
    {
      const double cost_estimate = top->get_total_cost_estimate();

      if (*_maximum_cost_estimate < cost_estimate)
        return true;
    }

    if (_interrupter && _interrupter())
    {
      _issues->interrupted = true;
      return true;
    }

    return false;
  }

  bool is_at_goal(const SearchNodePtr& top) const
  {
    if (top->waypoint == _goal_waypoint)
    {
      if (!_goal_yaw.has_value())
        return true;

      const double angle_diff = rmf_utils::wrap_to_pi(top->yaw - *_goal_yaw);
      if (std::abs(angle_diff) <= _rotation_threshold)
        return true;
    }

    return false;
  }

  bool is_finished(const SearchNodePtr& top) const
  {
    if (_goal_time.has_value())
    {
      if (top->time < *_goal_time)
        return false;
    }

    return is_at_goal(top);
  }

  void expand_start(const SearchNodePtr& top, SearchQueue& queue) const
  {
    const auto& start = top->start.value();
    const std::size_t target_waypoint_index = start.waypoint();
    const auto& wp = _supergraph->original().waypoints[target_waypoint_index];
    const Eigen::Vector2d wp_location = wp.get_location();

    SearchNodePtr parent = top;
    if (start.lane().has_value() && start.location().has_value())
    {
      const auto lane_index = start.lane().value();
      const Eigen::Vector2d p = start.location().value();
      const auto& lane = _supergraph->original().lanes[lane_index];
      if (lane.entry().event())
      {
        Graph::Lane::EventPtr entry_event = lane.entry().event()->clone();
        const double event_cost = time::to_seconds(entry_event->duration());
        Trajectory hold;
        Eigen::Vector3d p_start = {p.x(), p.y(), start.orientation()};
        const auto zero = Eigen::Vector3d::Zero();
        hold.insert(parent->time, p_start, zero);
        hold.insert(parent->time + entry_event->duration(), p_start, zero);
        Route route{wp.get_map_name(), std::move(hold)};
        if (_validator && !is_valid(parent, route))
        {
          // If we cannot wait for the entry event to happen then this is not a
          // feasible start.
          return;
        }

        parent = std::make_shared<SearchNode>(
          SearchNode{
            std::nullopt,
            top->waypoint,
            {},
            p,
            start.orientation(),
            parent->time + entry_event->duration(),
            parent->remaining_cost_estimate,
            {std::move(route)},
            std::move(entry_event),
            parent->current_cost + event_cost,
            std::nullopt,
            parent,
          __LINE__
          });
      }
    }

    Graph::Lane::EventPtr entry_event;
    double entry_event_cost = 0.0;
    Duration entry_event_duration = Duration(0);

    // If this start node did not have a waypoint, then it must have a location
    assert(start.location().has_value());

    const auto approach_info = make_start_approach_trajectories(
      top->start.value(), parent->current_cost);

    if (approach_info.trajectories.empty())
    {
      // This means there are no valid ways to approach the start. We should
      // just give up on expanding from this node.
      return;
    }

    std::vector<std::string> map_names;
    std::vector<std::size_t> approach_lanes;
    Graph::Lane::EventPtr exit_event;
    double exit_event_cost = 0.0;
    Duration exit_event_duration = Duration(0);
    if (const auto lane_index = start.lane())
    {
      approach_lanes.push_back(*lane_index);
      const auto& lane = _supergraph->original().lanes[*lane_index];

      const std::size_t wp0_index = lane.entry().waypoint_index();
      const auto& wp0 = _supergraph->original().waypoints[wp0_index];
      const auto& wp0_map = wp0.get_map_name();

      assert(lane.exit().waypoint_index() == target_waypoint_index);
      const auto& wp1_map = wp.get_map_name();

      map_names.push_back(wp0_map);
      if (wp0_map != wp1_map)
        map_names.push_back(wp1_map);

      if (lane.exit().event())
      {
        exit_event = lane.exit().event()->clone();
        exit_event_duration = exit_event->duration();
        exit_event_cost = time::to_seconds(exit_event_duration);
      }
    }
    else
    {
      map_names.push_back(wp.get_map_name());
    }

    for (const auto& approach_trajectory : approach_info.trajectories)
    {
      std::vector<Route> approach_routes;
      bool all_valid = true;

      for (const auto& map : map_names)
      {
        Route route{map, approach_trajectory};
        if (_validator && !is_valid(parent, route))
        {
          all_valid = false;
          break;
        }

        approach_routes.emplace_back(std::move(route));
      }

      if (!all_valid)
        continue;

      std::vector<Route> exit_event_routes;
      if (exit_event)
      {
        Trajectory hold;
        const auto& approached = approach_trajectory.back();
        hold.insert(approached);
        hold.insert(approached.time(), approached.position(), {0, 0, 0});

        bool all_valid = true;
        for (const auto& map : map_names)
        {
          Route route{map, hold};
          if (_validator && !is_valid(parent, route))
          {
            all_valid = false;
            break;
          }

          exit_event_routes.emplace_back(std::move(route));
        }

        if (!all_valid)
          continue;
      }

      const double approach_cost = calculate_cost(approach_trajectory);
      const double approach_yaw = approach_trajectory.back().position()[2];
      const auto approach_time = *approach_trajectory.finish_time();

      // TODO(MXG): We can actually specify the orientation for this. We just
      // need to be smarter with make_start_approach_trajectories(). We should
      // really have it return a Traversal.
      auto node = std::make_shared<SearchNode>(
        SearchNode{
          std::nullopt,
          target_waypoint_index,
          approach_lanes,
          wp_location,
          approach_yaw,
          approach_time,
          parent->remaining_cost_estimate - approach_cost,
          std::move(approach_routes),
          exit_event,
          parent->current_cost + approach_cost,
          std::nullopt,
          parent,
          __LINE__
        });

      if (exit_event)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            std::nullopt,
            target_waypoint_index,
            {},
            wp_location,
            approach_yaw,
            approach_time + exit_event_duration,
            node->remaining_cost_estimate - exit_event_cost,
            std::move(exit_event_routes),
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node,
          __LINE__
          });
      }

      queue.push(node);
    }

    const Time initial_time = top->time;
    const Time hold_until = initial_time + _holding_time;
    const double hold_cost = time::to_seconds(_holding_time);
    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    Trajectory hold;
    hold.insert(initial_time, position, zero);
    hold.insert(hold_until, position, zero);

    std::vector<Route> hold_routes;
    for (const auto& map : map_names)
    {
      Route route{map, hold};
      if (_validator && !is_valid(top, route))
        return;

      hold_routes.emplace_back(std::move(route));
    }

    queue.push(
      std::make_shared<SearchNode>(
        SearchNode{
          std::nullopt,
          top->waypoint,
          {},
          p,
          yaw,
          hold_until,
          top->remaining_cost_estimate,
          std::move(hold_routes),
          nullptr,
          top->current_cost + hold_cost,
          start,
          top,
          __LINE__
        }));
  }

  SearchNodePtr expand_hold(
    const SearchNodePtr& top,
    const Duration hold_time,
    const double cost_factor) const
  {
    const std::size_t wp_index = top->waypoint.value();
    if (_supergraph->original().waypoints[wp_index].is_passthrough_point())
      return nullptr;

    const std::string& map_name =
      _supergraph->original().waypoints[wp_index].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double yaw = top->yaw;
    const Eigen::Vector3d position{p.x(), p.y(), yaw};
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();
    const auto start_time = top->time;
    const auto finish_time = start_time + hold_time;
    const auto cost = cost_factor * time::to_seconds(hold_time);

    Trajectory trajectory;
    trajectory.insert(start_time, position, zero);
    trajectory.insert(finish_time, position, zero);

    Route route{map_name, std::move(trajectory)};

    if (!is_valid(top, route))
      return nullptr;

    return std::make_shared<SearchNode>(
      SearchNode{
        top->entry,
        wp_index,
        {},
        p,
        yaw,
        finish_time,
        top->remaining_cost_estimate,
        {std::move(route)},
        nullptr,
        top->current_cost + cost,
        std::nullopt,
        top,
          __LINE__
      });
  }

  void expand_hold(
    const SearchNodePtr& top,
    SearchQueue& queue) const
  {
    if (const auto node = expand_hold(top, _holding_time, 1.0))
    {
      if (_should_expand_to(node))
        queue.push(node);
    }
  }

  SearchNodePtr rotate_to_goal(const SearchNodePtr& top) const
  {
    assert(top->waypoint == _goal_waypoint);
    const std::string& map_name =
      _supergraph->original().waypoints[_goal_waypoint].get_map_name();

    const Eigen::Vector2d p = top->position;
    const double target_yaw = _goal_yaw.value();
    const Eigen::Vector3d start_position{p.x(), p.y(), top->yaw};
    const auto start_time = top->time;

    const Eigen::Vector3d finish_position{p.x(), p.y(), target_yaw};

    Trajectory trajectory;
    trajectory.insert(
      start_time, start_position, Eigen::Vector3d::Zero());
    internal::interpolate_rotation(
      trajectory, _w_nom, _alpha_nom, start_time,
      start_position, finish_position, _rotation_threshold);

    assert(trajectory.size() >= 2);

    const auto finish_time = *trajectory.finish_time();
    const double cost = calculate_cost(trajectory);
    Route route{map_name, std::move(trajectory)};
    if (_validator && !is_valid(top, route))
      return nullptr;

    return std::make_shared<SearchNode>(
      SearchNode{
        std::nullopt,
        _goal_waypoint,
        {},
        p,
        target_yaw,
        finish_time,
        0.0,
        {std::move(route)},
        nullptr,
        top->current_cost + cost,
        std::nullopt,
        top,
          __LINE__
      });
  }

  bool is_valid(const SearchNodePtr& parent, const Route& route) const
  {
    if (!_validator)
      return true;

    if (route.trajectory().size() >= 2)
    {
      auto conflict = _validator->find_conflict(route);
      if (conflict)
      {
        auto time_it =
          _issues->blocked_nodes[conflict->dependency.on_participant]
          .insert({parent, conflict->time});

        if (!time_it.second)
        {
          time_it.first->second =
            std::max(time_it.first->second, conflict->time);
        }

        return false;
      }
    }

    return true;
  }

  void expand_traversal(
    const SearchNodePtr& top,
    const Traversal& traversal,
    SearchQueue& queue) const
  {
    const auto initial_waypoint_index = top->waypoint.value();
    const auto& initial_waypoint =
      _supergraph->original().waypoints[initial_waypoint_index];
    const Eigen::Vector2d p0 = initial_waypoint.get_location();
    const double initial_yaw = top->yaw;
    const std::string& initial_map_name = initial_waypoint.get_map_name();

    const auto next_waypoint_index = traversal.finish_waypoint_index;
    const auto& next_waypoint =
      _supergraph->original().waypoints[next_waypoint_index];
    const Eigen::Vector2d next_position = next_waypoint.get_location();
    const std::string& next_map_name = next_waypoint.get_map_name();

    for (std::size_t i = 0; i < traversal.alternatives.size(); ++i)
    {
      const auto& alt = traversal.alternatives[i];

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << "Expanding from " << top->waypoint.value()
                << " -> " << traversal.finish_waypoint_index << " | "
                << Orientation(i) << " {" << traversal.entry_event << "}"
                << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      if (!alt.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== nullopt alternative" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const Orientation orientation = Orientation(i);

      Time start_time = top->time;
      const auto traversal_yaw = alt->yaw;

      Trajectory approach_trajectory;
      const Eigen::Vector3d start{p0.x(), p0.y(), initial_yaw};
      approach_trajectory.insert(
        start_time, start, Eigen::Vector3d::Zero());

      // TODO(MXG): We could push the logic for creating this trajectory
      // upstream into the traversal alternative.
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      double approach_cost = 0.0;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      if (traversal_yaw.has_value())
      {
        const Eigen::Vector3d finish{p0.x(), p0.y(), * traversal_yaw};
        internal::interpolate_rotation(
          approach_trajectory, _w_nom, _alpha_nom, start_time,
          start, finish, _rotation_threshold);

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        approach_cost = calculate_cost(approach_trajectory);
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      }

      auto approach_route =
        Route{
        initial_map_name,
        std::move(approach_trajectory)
      };

      if (!is_valid(top, approach_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid approach route" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      Trajectory entry_event_trajectory;
      const auto& approach_wp = approach_route.trajectory().back();
      entry_event_trajectory.insert(approach_wp);
      double entry_event_cost = 0.0;
      if (traversal.entry_event
        && traversal.entry_event->duration() > Duration(0))
      {
        const auto duration = traversal.entry_event->duration();
        entry_event_cost = time::to_seconds(duration);

        entry_event_trajectory.insert(
          approach_wp.time() + duration,
          approach_wp.position(), Eigen::Vector3d::Zero());
      }

      auto entry_event_route =
        Route{
        initial_map_name,
        std::move(entry_event_trajectory)
      };

      if (!is_valid(top, entry_event_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid entry event route" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const auto& ready_wp = entry_event_route.trajectory().back();
      const auto ready_time = ready_wp.time();
      const double ready_yaw = ready_wp.position()[2];
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      for (const auto& r : traversal_result.routes)
      {
        if (!is_valid(top, r))
        {
          all_valid = false;
          break;
        }
      }

      if (!all_valid)
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== Invalid traversal" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << " --------" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      const auto remaining_cost_estimate = _heuristic.compute(
        next_waypoint_index, traversal_result.finish_yaw);

      if (!remaining_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== nullopt heuristic" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      const auto& arrival_wp =
        traversal_result.routes.back().trajectory().back();

      Trajectory exit_event_trajectory;
      exit_event_trajectory.insert(arrival_wp);
      double exit_event_cost = 0.0;
      Duration exit_event_duration = Duration(0);
      if (traversal.exit_event
        && traversal.exit_event->duration() > Duration(0))
      {
        exit_event_duration = traversal.exit_event->duration();
        exit_event_cost = time::to_seconds(exit_event_duration);

        exit_event_trajectory.insert(
          arrival_wp.time() + exit_event_duration,
          arrival_wp.position(), Eigen::Vector3d::Zero());
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << "Cost " << approach_cost + entry_event_cost + alt->cost
        + exit_event_cost << " = " << "Approach: " << approach_cost
                << " | Entry: " << entry_event_cost << " | Alt: " << alt->cost
                << " | Exit: " << exit_event_cost << std::endl;
      std::cout << "Previous cost " << top->current_cost << " + Cost "
                << approach_cost + entry_event_cost + alt->cost
        + exit_event_cost << " = " << top->current_cost
        + approach_cost + entry_event_cost + alt->cost
        + exit_event_cost << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      auto exit_event_route =
        Route{
        next_map_name,
        std::move(exit_event_trajectory)
      };

      if (!is_valid(top, exit_event_route))
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " ==== invalid exit event" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        continue;
      }

      auto node = top;
      if (approach_route.trajectory().size() >= 2 || traversal.entry_event)
      {
        const double cost = calculate_cost(approach_route.trajectory());
        const double yaw = approach_wp.position()[2];
        const auto time = approach_wp.time();
        auto parent = node;
        std::vector<Route> route_from_parent;
        std::vector<std::size_t> approach_lanes;
        if (approach_route.trajectory().size() < 2)
        {
          // This is just an entry event so we will skip the unnecessary
          // intermediate node
          parent = node->parent;
          approach_lanes = node->approach_lanes;
          if (!parent)
          {
            // If the top node is a root node, then don't skip it
            parent = node;
          }
          route_from_parent = node->route_from_parent;
        }
        else
        {
          route_from_parent = {std::move(approach_route)};
        }

        node = std::make_shared<SearchNode>(
          SearchNode{
            Entry{
              traversal.initial_lane_index,
              orientation,
              Side::Start
            },
            initial_waypoint_index,
            approach_lanes,
            p0,
            yaw,
            time,
            *remaining_cost_estimate
            + entry_event_cost + alt->cost + exit_event_cost,
            route_from_parent,
            traversal.entry_event,
            node->current_cost + cost,
            std::nullopt,
            parent,
          __LINE__
          });
      }

      if (entry_event_route.trajectory().size() >= 2)
      {
        auto& front = traversal_result.routes.front();
        if (entry_event_route.map() == front.map())
        {
          for (const auto& wp : entry_event_route.trajectory())
            front.trajectory().insert(wp);
        }
        else
        {
          traversal_result.routes.insert(
            traversal_result.routes.begin(),
            entry_event_route);
        }
      }

      const Entry finish_key = Entry{
        traversal.initial_lane_index,
        orientation,
        Side::Finish
      };

      node = std::make_shared<SearchNode>(
        SearchNode{
          traversal.exit_event ? std::nullopt : std::make_optional(finish_key),
          next_waypoint_index,
          traversal.traversed_lanes,
          next_position,
          traversal_result.finish_yaw,
          traversal_result.finish_time,
          *remaining_cost_estimate + exit_event_cost,
          std::move(traversal_result.routes),
          traversal.exit_event,
          node->current_cost + entry_event_cost + alt->cost,
          std::nullopt,
          node,
          __LINE__
        });

      if (traversal.exit_event && exit_event_route.trajectory().size() >= 2)
      {
        node = std::make_shared<SearchNode>(
          SearchNode{
            finish_key,
            next_waypoint_index,
            {},
            next_position,
            traversal_result.finish_yaw,
            traversal_result.finish_time + exit_event_duration,
            *remaining_cost_estimate,
            {std::move(exit_event_route)},
            nullptr,
            node->current_cost + exit_event_cost,
            std::nullopt,
            node,
          __LINE__
          });
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << " ^^^^^^^^^^^^^^ Pushing" << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      if (_should_expand_to(node))
        queue.push(node);
    }
  }

  void expand_freely(
    const SearchNodePtr& top,
    SearchQueue& queue) const
  {
    // This function is used when there is no validator. We can just expand
    // freely to the goal without validating the results.
    const auto keys = _supergraph->keys_for(
      top->waypoint.value(), _goal_waypoint, _goal_yaw);

    for (const auto& key : keys)
    {
      const auto solution_root = _heuristic.cache().get(key);
      if (!solution_root)
      {
        // There is no solution for this key
        continue;
      }

      auto search_node = top;

      auto approach_info = solution_root->route_factory(top->time, top->yaw);
      if (approach_info.routes.back().trajectory().size() >= 2
        || solution_root->info.event)
      {
        search_node = std::make_shared<SearchNode>(
          SearchNode{
            solution_root->info.entry,
            solution_root->info.waypoint,
            solution_root->info.approach_lanes,
            solution_root->info.position,
            approach_info.finish_yaw,
            approach_info.finish_time,
            solution_root->info.remaining_cost_estimate,
            std::move(approach_info.routes),
            solution_root->info.event,
            search_node->current_cost + approach_info.cost,
            std::nullopt,
            search_node,
          __LINE__
          });
      }

      auto solution_node = solution_root->child;

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
//      std::cout << "Free solution: ";
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      while (solution_node)
      {
        assert(solution_node->route_factory);

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << "(" << search_node->current_cost << "; ";
        if (search_node->waypoint.has_value())
          std::cout << top->waypoint.value();
        else
          std::cout << "null";

        std::cout << ", " << search_node->yaw << ") ";
        if (solution_node->info.entry.has_value())
          std::cout << *solution_node->info.entry;
        else
          std::cout << "[null]";

        std::cout << " <" << solution_node->info.cost_from_parent
                  << " : " << solution_node->info.remaining_cost_estimate
                  << "> --> ";
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        auto route_info = solution_node->route_factory(
          search_node->time, search_node->yaw);

        search_node = std::make_shared<SearchNode>(
          SearchNode{
            solution_node->info.entry,
            solution_node->info.waypoint,
            solution_node->info.approach_lanes,
            solution_node->info.position,
            route_info.finish_yaw,
            route_info.finish_time,
            solution_node->info.remaining_cost_estimate,
            std::move(route_info.routes),
            solution_node->info.event,
            search_node->current_cost + solution_node->info.cost_from_parent,
            std::nullopt,
            search_node,
          __LINE__
          });

        solution_node = solution_node->child;
      }

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
      std::cout << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

      queue.push(search_node);
    }
  }

  void expand(const SearchNodePtr& top, SearchQueue& queue) const
  {
    if (!_should_expand_from(top))
    {
      // This means we have already expanded from this location before, at
      // approximately the same time, so there is no value in expanding this
      // again.
      return;
    }

    if (!top->waypoint.has_value())
    {
      // If the node does not have a waypoint, then it must be a start node.
      if (!top->start.has_value())
      {
        throw std::runtime_error(
          "[rmf_traffic::agv::planning::DifferentialDrivePlanner::expand] "
          "Node has no waypoint and also no start information. It was produced "
          "on line [" + std::to_string(top->line) + "]. This should not be "
          "possible. Please report this critical bug to the maintainers of "
          "rmf_traffic.");
      }
      expand_start(top, queue);
      return;
    }

    const auto current_wp_index = top->waypoint.value();
    if (is_at_goal(top))
    {
      // If there is no goal time, then is_finished should have caught this node
      assert(_goal_time.has_value());

      const Duration remaining_time = _goal_time.value() - top->time;

      const auto finishing_node = expand_hold(top, remaining_time, 0.0);
      if (finishing_node)
      {
        // If we can reach the finish time by just sitting here, then that will
        // surely be the optimal solution. We will simply push this new node
        // into the queue and return.
        queue.push(finishing_node);
        return;
      }

      // If we cannot hold all the way until the finishing time, then we will
      // do a zero-cost brief hold so that we spend as much time waiting on the
      // goal as allowed.
      if (const auto brief_hold = expand_hold(top, _holding_time, 0.0))
        queue.push(brief_hold);
    }
    else if (current_wp_index == _goal_waypoint)
    {
      // If there is no goal yaw, then is_at_goal should have caught this node
      assert(_goal_yaw.has_value());

      if (auto node = rotate_to_goal(top))
        queue.push(std::move(node));

      if (_validator)
        expand_hold(top, queue);
    }
    else if (_validator)
    {
      // There will never be a reason to hold if there is no validator.
      expand_hold(top, queue);
    }

    if (!_validator)
    {
      // If we don't have a validator, then we can jump straight to the solution
      expand_freely(top, queue);
      return;
    }

    const auto traversals = _supergraph->traversals_from(current_wp_index);
    for (const auto& traversal : *traversals)
      expand_traversal(top, traversal, queue);
  }

  struct ApproachInfo
  {
    bool need_approach;
    std::vector<Trajectory> trajectories;
  };

  ApproachInfo make_start_approach_trajectories(
    const Planner::Start& start,
    const double hold_time) const
  {
    // TODO(MXG): We could stash ApproachInfo into the start node to avoid
    // needing to regenerate these trajectories repeatedly.

    const auto location_opt = start.location();
    if (!location_opt.has_value())
      return {false, {}};

    // This will return all the different trajectories that can be used to
    // approach the start. If it returns empty, that means either you forgot to
    // check the location field, or there are no valid ways to approach the
    // start.

    const auto* differential = _supergraph->traits().get_differential();
    DifferentialDriveConstraint constraint{
      differential->get_forward(),
      differential->is_reversible()
    };

    const std::size_t waypoint_index = start.waypoint();

    const Eigen::Vector2d p0 = *location_opt;
    const Eigen::Vector2d p1 =
      _supergraph->original().waypoints[waypoint_index].get_location();

    const double translation_thresh = _supergraph->options().translation_thresh;

    const double dist = (p1 - p0).norm();
    if (dist < translation_thresh)
    {
      // No trajectory is really needed.
      return {false, {}};
    }

    const Eigen::Vector2d course_vector = (p1 - p0)/dist;
    const auto yaw_options = constraint.get_orientations(course_vector);

    const Graph::Lane* const lane = start.lane() ?
      &_supergraph->original().lanes[*start.lane()] : nullptr;
    const Graph::OrientationConstraint* const entry_constraint =
      lane ? lane->entry().orientation_constraint() : nullptr;
    const Graph::OrientationConstraint* const exit_constraint =
      lane ? lane->exit().orientation_constraint() : nullptr;

    const double rotation_thresh = _supergraph->options().rotation_thresh;
    const auto start_time = start.time() + time::from_seconds(hold_time);
    const double start_yaw = start.orientation();
    const Eigen::Vector3d zero = Eigen::Vector3d::Zero();

    const auto& traits = _supergraph->traits();
    const auto& linear = traits.linear();
    const auto& angular = traits.rotational();
    const double v_nom = linear.get_nominal_velocity();
    const double a_nom = linear.get_nominal_acceleration();
    const double w_nom = angular.get_nominal_velocity();
    const double alpha_nom = angular.get_nominal_acceleration();

    std::vector<Trajectory> trajectories;
    for (const auto& yaw_opt : yaw_options)
    {
      if (!yaw_opt.has_value())
        continue;

      const double yaw = *yaw_opt;

      if (!orientation_constraint_satisfied(
          p0, yaw, course_vector, entry_constraint, rotation_thresh))
        continue;

      if (!orientation_constraint_satisfied(
          p1, yaw, course_vector, exit_constraint, rotation_thresh))
        continue;

      Trajectory trajectory;
      const Eigen::Vector3d p_start = {p0.x(), p0.y(), start_yaw};
      trajectory.insert(start_time, p_start, zero);

      const Eigen::Vector3d p_oriented{p0.x(), p0.y(), yaw};
      internal::interpolate_rotation(
        trajectory, w_nom, alpha_nom, start_time, p_start, p_oriented,
        rotation_thresh);

      const Eigen::Vector3d p_arrived{p1.x(), p1.y(), yaw};
      internal::interpolate_translation(
        trajectory, v_nom, a_nom, *trajectory.finish_time(),
        p_oriented, p_arrived, translation_thresh);

      trajectories.emplace_back(std::move(trajectory));
    }

    return {true, std::move(trajectories)};
  }

  SearchNodePtr make_start_node(const Planner::Start& start) const
  {
    const std::size_t initial_waypoint_index = start.waypoint();
    const auto& initial_waypoint =
      _supergraph->original().waypoints.at(initial_waypoint_index);
    const auto& initial_map = initial_waypoint.get_map_name();

    const auto initial_time = start.time();
    const auto initial_yaw = start.orientation();
    double remaining_cost_estimate = 0.0;
    std::optional<std::size_t> node_waypoint;

    const Eigen::Vector2d waypoint_location =
      _supergraph->original().waypoints[initial_waypoint_index].get_location();

#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
    std::cout << " >> Making start node " << start.waypoint() << ", "
              << start.orientation() << std::endl;
    if (start.location().has_value())
    {
      std::cout << "   > location: " << start.location().value().transpose()
                << std::endl;
    }
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

    Trajectory start_point_trajectory;
    const auto start_location = start.location();

    auto approach_info = make_start_approach_trajectories(start, 0.0);
    if (approach_info.need_approach)
    {
      std::optional<double> lowest_cost_estimate;
      for (const auto& approach : approach_info.trajectories)
      {
        const double yaw = approach.back().position()[2];
        double cost = calculate_cost(approach);
        const auto heuristic_cost_estimate =
          _heuristic.compute(initial_waypoint_index, yaw);

        if (!heuristic_cost_estimate.has_value())
          continue;

        cost += *heuristic_cost_estimate;
        if (!lowest_cost_estimate.has_value() || cost < *lowest_cost_estimate)
          lowest_cost_estimate = cost;
      }

      if (!lowest_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " === nullopt heuristic " << __LINE__ << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *lowest_cost_estimate;

      if (const auto lane_index = start.lane())
      {
        const auto& lane = _supergraph->original().lanes[*lane_index];
        if (const auto* exit_event = lane.exit().event())
          remaining_cost_estimate += time::to_seconds(exit_event->duration());
      }

      const Eigen::Vector3d start_position{
        start_location->x(),
        start_location->y(),
        initial_yaw
      };
      start_point_trajectory.insert(initial_time, start_position, {0, 0, 0});
    }
    else
    {
      node_waypoint = initial_waypoint_index;
      const auto heuristic_cost_estimate =
        _heuristic.compute(initial_waypoint_index, initial_yaw);

      if (!heuristic_cost_estimate.has_value())
      {
#ifdef RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER
        std::cout << " === nullopt heuristic " << __LINE__ << std::endl;
#endif // RMF_TRAFFIC__AGV__PLANNING__DEBUG__PLANNER

        // If this happens, the heuristic found that there is simply no path
        // from this start to the goal, so we return a nullptr.
        return nullptr;
      }

      remaining_cost_estimate = *heuristic_cost_estimate;

      const Eigen::Vector3d start_position{
        waypoint_location.x(),
        waypoint_location.y(),
        initial_yaw
      };
      start_point_trajectory.insert(initial_time, start_position, {0, 0, 0});
    }

    assert(!start_point_trajectory.empty());

    return std::make_shared<SearchNode>(
      SearchNode{
        std::nullopt,
        node_waypoint,
        {},
        start_location.value_or(waypoint_location),
        initial_yaw,
        start.time(),
        remaining_cost_estimate,
        {{initial_map, std::move(start_point_trajectory)}},
        nullptr,
        0.0,
        start,
        nullptr,
          __LINE__
      });
  }

  struct RolloutEntry
  {
    Time initial_time;
    SearchNodePtr node;

    bool operator==(const RolloutEntry& r) const
    {
      return node == r.node;
    }

    Duration span() const
    {
      return *node->route_from_parent.back().trajectory().finish_time()
        - initial_time;
    }
  };

  bool is_holding_point(const std::optional<std::size_t> waypoint_index) const
  {
    if (!waypoint_index.has_value())
      return false;

    return _supergraph->original().waypoints
      .at(*waypoint_index).is_holding_point();
  }

  std::vector<schedule::Itinerary> rollout(
    const Duration max_span,
    const Issues::BlockedNodes& nodes,
    std::optional<std::size_t> max_rollouts) const
  {
    std::vector<RolloutEntry> rollout_queue;
    for (const auto& void_node : nodes)
    {
      bool skip = false;
      auto original_node =
        std::static_pointer_cast<SearchNode>(void_node.first);

      const auto original_t = void_node.second;

      // TODO(MXG): This filtering approach is not reliable as it could be.
      // Certain blockages will only be expanded half as far past the blockage
      // as the API implies. It would be better if each type of node expansion
      // could have a unique identifier so we could both avoid redundant
      // expansions while still expanding a blockage out as far as the API
      // says that it will.
//      const auto merge_span = max_span/2.0;

      auto ancestor = original_node->parent;
      while (ancestor)
      {
        if (nodes.count(ancestor) > 0)
        {
          // TODO(MXG): Consider if we should account for the time difference
          // between these conflicts so that we get a broader rollout.
//          const auto t = *ancestor->route_from_parent.trajectory.finish_time();
//          if (t - original_t < merge_span)
          {
            skip = true;
          }

          break;
        }

        ancestor = ancestor->parent;
      }

      if (skip)
        continue;

      while (original_node && !original_node->waypoint.has_value() && !original_node->start.has_value())
      {
        original_node = original_node->parent;
      }

      if (!original_node)
        continue;

      rollout_queue.emplace_back(
        RolloutEntry{
          original_t,
          original_node
        });

      // TODO(MXG): Consider making this configurable, or making a more
      // meaningful decision on how to prune the initial rollout queue.
      if (rollout_queue.size() > 5)
        break;
    }

    std::unordered_map<NodePtr, ConstRoutePtr> route_map;
    std::vector<schedule::Itinerary> alternatives;

    Issues::BlockerMap temp_blocked_nodes;

    SearchQueue search_queue;
    SearchQueue finished_rollouts;

    while (!rollout_queue.empty() && !(_interrupter && _interrupter()))
    {
      const auto top = rollout_queue.back();
      rollout_queue.pop_back();

      const auto current_span = top.span();

      const bool stop_expanding =
        (max_span < current_span)
        || is_finished(top.node)
        || is_holding_point(top.node->waypoint);

      if (stop_expanding)
      {
        finished_rollouts.push(top.node);

        if (max_rollouts && *max_rollouts <= finished_rollouts.size())
          break;

        continue;
      }

      expand(top.node, search_queue);
      while (!search_queue.empty())
      {
        rollout_queue.emplace_back(
          RolloutEntry{
            top.initial_time,
            search_queue.top()
          });

        search_queue.pop();
      }
    }

    while (!finished_rollouts.empty())
    {
      auto node = finished_rollouts.top();
      finished_rollouts.pop();

      auto [routes, _] = reconstruct_waypoints(
        reconstruct_nodes(node),
        _supergraph->original(),
        nullptr,
        std::nullopt,
        Duration(0),
        max_span);
      assert(!routes.empty());
      alternatives.emplace_back(std::move(routes));
    }

    return alternatives;
  }

  ScheduledDifferentialDriveExpander(
    State::Internal* internal,
    Issues& issues,
    std::shared_ptr<const Supergraph> supergraph,
    DifferentialDriveHeuristicAdapter heuristic,
    const Planner::Goal& goal,
    const Planner::Options& options,
    double traversal_cost_per_meter)
  : _internal(static_cast<InternalState*>(internal)),
    _issues(&issues),
    _supergraph(std::move(supergraph)),
    _heuristic(std::move(heuristic)),
    _goal_waypoint(goal.waypoint()),
    _goal_yaw(rmf_utils::pointer_to_opt(goal.orientation())),
    _goal_time(goal.minimum_time()),
    _validator(options.validator().get()),
    _holding_time(options.minimum_holding_time()),
    _discrete_time_window(_holding_time/2),
    _saturation_limit(options.saturation_limit()),
    _maximum_cost_estimate(options.maximum_cost_estimate()),
    _interrupter(options.interrupter()),
    _dependency_window(options.dependency_window()),
    _dependency_resolution(options.dependency_resolution()),
    _traversal_cost_per_meter(traversal_cost_per_meter),
    _already_expanded(4093, EntryHash(_supergraph->original().lanes.size()))
  {
    const auto& angular = _supergraph->traits().rotational();
    _w_nom = angular.get_nominal_velocity();
    _alpha_nom = angular.get_nominal_acceleration();
    _rotation_threshold = _supergraph->options().rotation_thresh;
  }

  class Debugger : public Interface::Debugger
  {
  public:
    const Planner::Debug::Node::SearchQueue& queue() const final
    {
      return queue_;
    }

    const std::vector<Planner::Debug::ConstNodePtr>&
    expanded_nodes() const final
    {
      return expanded_nodes_;
    }

    const std::vector<Planner::Debug::ConstNodePtr>&
    terminal_nodes() const final
    {
      return terminal_nodes_;
    }

    NodePtr convert(agv::Planner::Debug::ConstNodePtr from)
    {
      auto output = _from_debug[from];
      assert(output);

      return output;
    }

    Planner::Debug::ConstNodePtr convert(NodePtr from)
    {
      const auto it = _to_debug.find(from);
      if (it != _to_debug.end())
        return it->second;

      std::vector<NodePtr> queue;
      queue.push_back(from);
      while (!queue.empty())
      {
        const auto node = queue.back();

        const auto parent = node->parent;
        agv::Planner::Debug::ConstNodePtr debug_parent = nullptr;
        if (parent)
        {
          const auto parent_it = _to_debug.find(parent);
          if (parent_it == _to_debug.end())
          {
            queue.push_back(parent);
            continue;
          }

          debug_parent = parent_it->second;
        }

        auto new_debug_node = std::make_shared<Planner::Debug::Node>(
          agv::Planner::Debug::Node{
            debug_parent,
            node->route_from_parent,
            node->remaining_cost_estimate,
            node->current_cost,
            node->waypoint,
            node->yaw,
            node->event,
            std::nullopt,
            next_id++
          });

        _to_debug[node] = new_debug_node;
        _from_debug[new_debug_node] = node;
        queue.pop_back();
      }

      return _to_debug[from];
    }

    agv::Planner::Debug::Node::SearchQueue queue_;
    std::vector<agv::Planner::Debug::ConstNodePtr> expanded_nodes_;
    std::vector<agv::Planner::Debug::ConstNodePtr> terminal_nodes_;
    Issues::BlockerMap blocked_nodes_;

    std::vector<agv::Planner::Start> starts_;
    agv::Planner::Goal goal_;
    agv::Planner::Options options_;

    std::size_t next_id = 0;

    Debugger(
      std::vector<agv::Planner::Start> starts,
      agv::Planner::Goal goal,
      agv::Planner::Options options)
    : starts_(std::move(starts)),
      goal_(std::move(goal)),
      options_(std::move(options))
    {
      // Do nothing
    }

    std::optional<PlanData> step(
      std::shared_ptr<const Supergraph> supergraph,
      Cache<DifferentialDriveHeuristic> cache)
    {
      InternalState internal;
      Issues issues;

      ScheduledDifferentialDriveExpander expander{
        &internal,
        issues,
        supergraph,
        DifferentialDriveHeuristicAdapter{
          cache,
          supergraph,
          goal_.waypoint(),
          rmf_utils::pointer_to_opt(goal_.orientation())
        },
        goal_,
        options_,
        supergraph->traversal_cost_per_meter()
      };

      return expander.debug_step(*this);
    }

  private:
    std::unordered_map<Planner::Debug::ConstNodePtr, NodePtr> _from_debug;
    std::unordered_map<NodePtr, Planner::Debug::ConstNodePtr> _to_debug;
  };

  std::unique_ptr<Interface::Debugger> debug_begin(
    const std::vector<agv::Planner::Start>& starts,
    agv::Planner::Goal goal,
    agv::Planner::Options options) const
  {
    auto debugger = std::make_unique<Debugger>(
      starts,
      std::move(goal),
      std::move(options));

    for (const auto& start : starts)
    {
      if (auto start_node = make_start_node(start))
        debugger->queue_.push(debugger->convert(std::move(start_node)));
    }

    return debugger;
  }

  std::optional<PlanData> debug_step(
    Interface::Debugger& input_debugger) const
  {
    Debugger& debugger = static_cast<Debugger&>(input_debugger);

    if (debugger.queue_.empty())
      return std::nullopt;

    auto top = debugger.convert(debugger.queue_.top());
    debugger.queue_.pop();
    debugger.expanded_nodes_.push_back(debugger.convert(top));

    if (is_finished(top))
      return make_plan(top);

    SearchQueue queue;
    expand(top, queue);

    if (queue.empty())
    {
      debugger.terminal_nodes_.push_back(debugger.convert(top));
    }
    else
    {
      while (!queue.empty())
      {
        debugger.queue_.push(debugger.convert(queue.top()));
        queue.pop();
      }
    }

    return std::nullopt;
  }

  PlanData make_plan(const SearchNodePtr& solution) const
  {
    auto nodes = reconstruct_nodes(
      solution, _validator, _w_nom, _alpha_nom, _rotation_threshold);

    auto [routes, waypoints] = reconstruct_waypoints(
      nodes,
      _supergraph->original(),
      _validator,
      _dependency_window,
      _dependency_resolution);

    auto start = find_start(solution);

    return PlanData{
      std::move(routes),
      std::move(waypoints),
      std::move(start),
      solution->current_cost
    };
  }

  double calculate_cost(const rmf_traffic::Trajectory& traj) const
  {
    return planning::calculate_cost(traj, _traversal_cost_per_meter);
  }

private:
  InternalState* _internal;
  Issues* _issues;
  std::shared_ptr<const Supergraph> _supergraph;
  DifferentialDriveHeuristicAdapter _heuristic;
  std::size_t _goal_waypoint;
  std::optional<double> _goal_yaw;
  std::optional<rmf_traffic::Time> _goal_time;
  const RouteValidator* _validator;
  Duration _holding_time;
  Duration _discrete_time_window;
  std::optional<std::size_t> _saturation_limit;
  std::optional<double> _maximum_cost_estimate;
  std::function<bool()> _interrupter;
  std::optional<Duration> _dependency_window;
  Duration _dependency_resolution;
  double _w_nom;
  double _alpha_nom;
  double _rotation_threshold;
  double _traversal_cost_per_meter;

  using TimeSet = std::set<rmf_traffic::Time>;
  using VisitMap = std::unordered_map<
    DifferentialDriveMapTypes::Entry,
    TimeSet,
    DifferentialDriveMapTypes::EntryHash
  >;

  mutable VisitMap _already_expanded;

  std::optional<TimeSet::const_iterator> _get_hint_if_not_redundant(
    const Time time,
    const TimeSet& time_set) const
  {
    const auto greater_or_equal_it = time_set.lower_bound(time);
    if (greater_or_equal_it == time_set.begin())
    {
      if (time + _discrete_time_window < *greater_or_equal_it)
      {
        return greater_or_equal_it;
      }

      // The node's time is within the tolerance of one that was already
      // expanded
      return std::nullopt;
    }

    const auto less_than = --TimeSet::const_iterator(greater_or_equal_it);
    if (time < *less_than + _discrete_time_window)
    {
      // The node is within the tolerance of an earlier time
      return std::nullopt;
    }

    // If we reach this point, the node is far enough from the earlier time, so
    // now we need to check if it's far enough from any later time.

    if (greater_or_equal_it == time_set.end())
    {
      // There is no need to check the node against a later time, because there
      // is no later time in the set.
      return greater_or_equal_it;
    }

    if (time + _discrete_time_window < *greater_or_equal_it)
    {
      return greater_or_equal_it;
    }

    return std::nullopt;
  }

  bool _should_expand_from(const SearchNodePtr& node) const
  {
    if (!node->entry.has_value())
      return true;

    const auto entry_it = _already_expanded.insert({*node->entry, {}}).first;
    TimeSet& time_set = entry_it->second;
    if (time_set.empty())
    {
      time_set.insert(node->time);
      return true;
    }

    const auto time = node->time;
    const auto hint = _get_hint_if_not_redundant(time, time_set);
    if (hint.has_value())
    {
      time_set.insert(*hint, time);
      return true;
    }

    return false;
  }

  bool _should_expand_to(const SearchNodePtr& node) const
  {
    if (!node->entry.has_value())
      return true;

    const auto entry_it = _already_expanded.insert({*node->entry, {}}).first;
    const TimeSet& time_set = entry_it->second;
    if (time_set.empty())
      return true;

    return _get_hint_if_not_redundant(node->time, time_set).has_value();
  }
};

//==============================================================================
DifferentialDrivePlanner::DifferentialDrivePlanner(
  Planner::Configuration config)
: _config(std::move(config))
{
  _supergraph = Supergraph::make(
    Graph::Implementation::get(_config.graph()),
    _config.vehicle_traits(),
    _config.lane_closures(),
    _config.interpolation(),
    _config.traversal_cost_per_meter());

  _shortest_path = std::make_shared<ShortestPathHeuristic>(_supergraph);

  _cache = DifferentialDriveHeuristic::make_manager(_supergraph, _shortest_path);
}

//==============================================================================
State DifferentialDrivePlanner::initiate(
  const std::vector<Planner::Start>& starts,
  Planner::Goal input_goal,
  Planner::Options options) const
{
  using InternalState = ScheduledDifferentialDriveExpander::InternalState;

  State state{
    Conditions{
      starts,
      std::move(input_goal),
      std::move(options)
    },
    Issues{},
    std::nullopt,
    rmf_utils::make_derived_impl<State::Internal, InternalState>()
  };

  auto& internal = static_cast<InternalState&>(*state.internal);
  const auto& goal = state.conditions.goal;

  ScheduledDifferentialDriveExpander expander{
    state.internal.get(),
    state.issues,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    goal,
    state.conditions.options,
    _supergraph->traversal_cost_per_meter()
  };

  for (const auto& start : starts)
  {
    if (auto node = expander.make_start_node(start))
      internal.queue.push(node);
  }

  if (internal.queue.empty())
  {
    state.issues.disconnected = true;
  }
  else
  {
    const auto& top = internal.queue.top();
    state.ideal_cost = top->get_total_cost_estimate();
  }

  return state;
}

//==============================================================================
std::optional<PlanData> DifferentialDrivePlanner::plan(State& state) const
{
  const auto& goal = state.conditions.goal;

  ScheduledDifferentialDriveExpander expander{
    state.internal.get(),
    state.issues,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    state.conditions.goal,
    state.conditions.options,
    _supergraph->traversal_cost_per_meter()
  };

  using InternalState = ScheduledDifferentialDriveExpander::InternalState;
  auto& internal = static_cast<InternalState&>(*state.internal);

  const auto solution = a_star_search(expander, internal.queue);

  if (!solution)
    return std::nullopt;

  return expander.make_plan(solution);
}

//==============================================================================
std::vector<schedule::Itinerary> DifferentialDrivePlanner::rollout(
  const Duration span,
  const Issues::BlockedNodes& nodes,
  const Planner::Goal& goal,
  const Planner::Options& options,
  std::optional<std::size_t> max_rollouts) const
{
  using InternalState = ScheduledDifferentialDriveExpander::InternalState;
  InternalState internal;
  Issues issues;

  ScheduledDifferentialDriveExpander expander{
    &internal,
    issues,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation()),
    },
    goal,
    options,
    _supergraph->traversal_cost_per_meter()
  };

  return expander.rollout(span, nodes, max_rollouts);
}

//==============================================================================
std::optional<Planner::QuickestPath> DifferentialDrivePlanner::quickest_path(
  const Planner::StartSet& start_options,
  std::size_t goal_vertex) const
{
  std::optional<Planner::QuickestPath::Implementation> best;
  for (const auto& start : start_options)
  {
    const auto cost_offset = [&]()
      {
        const auto location = start.location();
        if (!location.has_value())
          return 0.0;

        const Eigen::Vector2d wp_location =
          _supergraph->original().waypoints.at(start.waypoint()).get_location();

        const auto speed = [&]()
          {
            const auto agent_speed =
              _supergraph->traits().linear().get_nominal_velocity();

            const auto lane_index = start.lane();
            if (!lane_index.has_value())
              return agent_speed;

            const auto& lane = _supergraph->original().lanes.at(*lane_index);
            const auto speed_limit = lane.properties().speed_limit();
            if (!speed_limit.has_value())
              return agent_speed;

            return std::min(agent_speed, *speed_limit);
          }();

        return (*location - wp_location).norm() / speed;
      }();

    const auto solution =
      _cache->inner()->inner_heuristic(start.waypoint(), goal_vertex);

    if (!solution)
      continue;

    Planner::QuickestPath::Implementation::choose_better(
      best,
      Planner::QuickestPath::Implementation{solution, cost_offset});
  }

  return Planner::QuickestPath::Implementation::promote(best);
}

//==============================================================================
const Planner::Configuration&
DifferentialDrivePlanner::get_configuration() const
{
  return _config;
}

//==============================================================================
auto DifferentialDrivePlanner::debug_begin(
  const std::vector<Planner::Start>& starts,
  Planner::Goal goal,
  Planner::Options options) const -> std::unique_ptr<Debugger>
{
  using InternalState = ScheduledDifferentialDriveExpander::InternalState;
  InternalState internal;
  Issues issues;

  ScheduledDifferentialDriveExpander expander{
    &internal,
    issues,
    _supergraph,
    DifferentialDriveHeuristicAdapter{
      _cache->get(),
      _supergraph,
      goal.waypoint(),
      rmf_utils::pointer_to_opt(goal.orientation())
    },
    goal,
    options,
    _supergraph->traversal_cost_per_meter()
  };

  return expander.debug_begin(starts, goal, options);
}

//==============================================================================
std::optional<PlanData> DifferentialDrivePlanner::debug_step(
  Debugger& input_debugger) const
{
  return static_cast<ScheduledDifferentialDriveExpander::Debugger&>(
    input_debugger).step(_supergraph, _cache->get());
}

//==============================================================================
Planner::CacheAudit DifferentialDrivePlanner::cache_audit() const
{
  auto audit = Planner::CacheAudit::Implementation{
    _cache->get().size(),
    _shortest_path->cache_size(),
    _shortest_path->heuristic_cache_size()
  };

  return Planner::CacheAudit::Implementation::make(audit);
}

//==============================================================================
void DifferentialDrivePlanner::clear_cache() const
{
  _cache->get().clear();
}

} // namespace planning
} // namespace agv
} // namespace rmf_traffic
