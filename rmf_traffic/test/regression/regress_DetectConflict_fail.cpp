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

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/agv/Graph.hpp>

#include <rmf_utils/catch.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace {

//==============================================================================
SCENARIO("Failed Detect Conflict")
{
  rmf_traffic::agv::VehicleTraits traits{
    {0.5, 2.0}, {0.75, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex(
        rmf_traffic::geometry::Circle(0.2))
    }
  };

  auto graph = rmf_traffic::agv::Graph();

  graph.add_waypoint("test", Eigen::Vector2d(66.4517, -8.31925));
  graph.add_waypoint("test", Eigen::Vector2d(66.4694, -11.86215));
  graph.add_waypoint("test", Eigen::Vector2d(63.575, -11.875));
  graph.add_waypoint("test", Eigen::Vector2d(16.097900000000003, -12.8518));
  graph.add_waypoint("test", Eigen::Vector2d(13.2777, -12.980850000000002));
  graph.add_waypoint("test", Eigen::Vector2d(13.325, -8.2798));
  graph.add_waypoint("test",
    Eigen::Vector2d(117.30075000000001, -12.863300000000002));
  graph.add_waypoint("test", Eigen::Vector2d(120.03815, -12.8944));
  graph.add_waypoint("test", Eigen::Vector2d(120.05175, -8.3695));
  graph.add_waypoint("test",
    Eigen::Vector2d(13.675250000000002, -28.884750000000004));
  graph.add_waypoint("test", Eigen::Vector2d(66.48685, -28.959050000000005));
  graph.add_waypoint("test", Eigen::Vector2d(119.97360000000002, -28.97215));
  graph.add_waypoint("test", Eigen::Vector2d(18.28865, -8.2941));
  graph.add_waypoint("test", Eigen::Vector2d(24.8581, -8.2971));
  graph.add_waypoint("test", Eigen::Vector2d(30.8229, -8.302249999999999));
  graph.add_waypoint("test", Eigen::Vector2d(38.977000000000004, -8.3191));
  graph.add_waypoint("test", Eigen::Vector2d(44.60830000000001, -8.324));
  graph.add_waypoint("test", Eigen::Vector2d(52.278800000000004, -8.3107));
  graph.add_waypoint("test", Eigen::Vector2d(63.325, -8.3315));

  graph.add_lane(0, 1);
  graph.add_lane(1, 0);
  graph.add_lane(1, 2);
  graph.add_lane(2, 1);
  graph.add_lane(3, 4);
  graph.add_lane(4, 3);
  graph.add_lane(4, 5);
  graph.add_lane(5, 4);
  graph.add_lane(6, 7);
  graph.add_lane(7, 6);
  graph.add_lane(7, 8);
  graph.add_lane(8, 7);
  graph.add_lane(4, 9);
  graph.add_lane(9, 4);
  graph.add_lane(1, 10);
  graph.add_lane(10, 1);
  graph.add_lane(0, 8);
  graph.add_lane(8, 0);
  graph.add_lane(7, 11);
  graph.add_lane(11, 7);
  graph.add_lane(5, 12);
  graph.add_lane(12, 13);
  graph.add_lane(13, 14);
  graph.add_lane(14, 15);
  graph.add_lane(15, 16);
  graph.add_lane(16, 17);
  graph.add_lane(17, 18);
  graph.add_lane(18, 0);
  graph.add_lane(0, 18);
  graph.add_lane(18, 17);
  graph.add_lane(17, 16);
  graph.add_lane(16, 15);
  graph.add_lane(15, 14);
  graph.add_lane(14, 13);
  graph.add_lane(13, 12);
  graph.add_lane(12, 5);

  const auto start_time = std::chrono::steady_clock::now();

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();

  rmf_traffic::agv::Planner planner_obstacle = rmf_traffic::agv::Planner{
    {graph, traits},
    {nullptr}
  };

  const auto N = database->participant_ids().size();

  auto new_obstacle = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle_" + std::to_string(N),
      "obstacles",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      planner_obstacle.get_configuration().vehicle_traits().profile()
    }, database);

  auto plan_obstacle =
    planner_obstacle.plan({rmf_traffic::time::apply_offset(start_time,
        106.362), 1, 90.0}, {5});

  new_obstacle.set(plan_obstacle->get_itinerary());

  const auto obstacle_validator =
    rmf_traffic::agv::ScheduleRouteValidator::make(
    database, std::numeric_limits<std::size_t>::max(), traits.profile());

  rmf_traffic::agv::Planner::Options options(obstacle_validator);
  options.saturation_limit(100000);
  rmf_traffic::agv::Planner planner = rmf_traffic::agv::Planner{
    {graph, traits},
    options
  };
  auto plan = planner.plan({start_time, 5, 180.0}, {2});

  CHECK_FALSE(plan.success());

  if (plan.success())
  {
    // The plan should not be successful, but these additional tests are to
    // highlight that simply changing the precision of the waypoints is
    // impacting the detection of the conflict.
    //
    // DetectConflict does not see any problem with the trajectory returned by
    // the planner, but if we merely reduce its precision slightly by casting it
    // to single-precision floating point values, suddenly the collision gets
    // detected.
    for (const auto& agent_route : plan->get_itinerary())
    {
      for (const auto& obstacle_route : new_obstacle.itinerary())
      {
        CHECK_FALSE(rmf_traffic::DetectConflict::between(
                traits.profile(), agent_route.trajectory(),
                traits.profile(), obstacle_route.route->trajectory()));

        auto low_precision_agent_trajectory = agent_route.trajectory();
        for (auto& wp : low_precision_agent_trajectory)
          wp.position(wp.position().cast<float>().cast<double>());

        CHECK_FALSE(rmf_traffic::DetectConflict::between(
                traits.profile(), low_precision_agent_trajectory,
                traits.profile(), obstacle_route.route->trajectory()));
      }
    }
  }
}
}
