/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rmf_traffic/agv/CentralizedNegotiation.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <iostream>
#include <map>

//==============================================================================
struct Scenario
{
  rmf_traffic::agv::Graph graph;
  std::shared_ptr<rmf_traffic::schedule::Database> database;
  rmf_traffic::schedule::Participant R0;
  rmf_traffic::schedule::Participant R1;
  rmf_traffic::schedule::Participant R2;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
};

using CentralizedNegotiation = rmf_traffic::agv::CentralizedNegotiation;
using Start = rmf_traffic::agv::Plan::Start;
using Goal = rmf_traffic::agv::Plan::Goal;

//==============================================================================
rmf_traffic::agv::Graph make_graph()
{

  const std::string map = "test_map_name";

  /*
   *              Top 9
   *                  |
   *                  8
   *                  |
   *                  7
   *                  |
   * Left 0---1---2---3---4---5---6 Right
   *                Middle
   */

  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(map, {0.0, 0.0}); // 0
  graph.add_waypoint(map, {1.0, 0.0}); // 1
  graph.add_waypoint(map, {2.0, 0.0}); // 2
  graph.add_waypoint(map, {3.0, 0.0}); // 3
  graph.add_waypoint(map, {4.0, 0.0}); // 4
  graph.add_waypoint(map, {5.0, 0.0}); // 5
  graph.add_waypoint(map, {6.0, 0.0}); // 6
  graph.add_waypoint(map, {3.0, 1.0}); // 7
  graph.add_waypoint(map, {3.0, 2.0}); // 8
  graph.add_waypoint(map, {3.0, 3.0}); // 9

  graph.add_key("Left", 0);
  graph.add_key("Middle", 3);
  graph.add_key("Right", 6);
  graph.add_key("Top", 9);

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(2, 3);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(3, 7);
  add_bidir_lane(7, 8);
  add_bidir_lane(8, 9);

  return graph;
}

//==============================================================================
Scenario make_scenario()
{
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(0.1)
  };

  rmf_traffic::schedule::Participant R0 =
    rmf_traffic::schedule::make_participant(
    {
      "R0",
      "robots",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  rmf_traffic::schedule::Participant R1 =
    rmf_traffic::schedule::make_participant(
    {
      "R1",
      "robots",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  rmf_traffic::schedule::Participant R2 =
    rmf_traffic::schedule::make_participant(
    {
      "R2",
      "robots",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database);

  rmf_traffic::agv::VehicleTraits traits{
    // Translational motion
    rmf_traffic::agv::VehicleTraits::Limits(0.7, 0.75),
    // Rotational motion
    rmf_traffic::agv::VehicleTraits::Limits(0.6, 2.0),
    // Geometry
    profile
  };

  auto graph = make_graph();
  auto planner = std::make_shared<rmf_traffic::agv::Planner>(
    rmf_traffic::agv::Planner::Configuration{graph, traits},
    rmf_traffic::agv::Planner::Options{nullptr});

  return Scenario{
    std::move(graph),
    std::move(database),
    std::move(R0),
    std::move(R1),
    std::move(R2),
    std::move(planner)
  };
}

//==============================================================================
void print_result(
  const Scenario& scenario,
  const rmf_traffic::agv::CentralizedNegotiation::Result& result)
{
  if (!result.proposal().has_value())
  {
    std::cout << "Failed to find a solution!" << std::endl;
    return;
  }

  // Switch to an ordered map
  std::map<rmf_traffic::schedule::ParticipantId, rmf_traffic::agv::Plan>
    proposal(result.proposal()->begin(), result.proposal()->end());

  for (const auto& [robot, plan] : proposal)
  {
    std::cout << "\n" << scenario.database->get_participant(robot)->name() << ":";
    for (const auto& wp : plan.get_waypoints())
    {
      const auto t = wp.time().time_since_epoch();
      std::cout << " { t=" << rmf_traffic::time::to_seconds(t);

      if (wp.graph_index().has_value())
      {
        const auto gi = *wp.graph_index();
        std::cout << ", wp=" << scenario.graph.get_waypoint(gi).name_or_index();
      }
      else
      {
        std::cout << ", p=(" << wp.position().block<2, 1>(0, 0).transpose() << ")";
      }

      std::cout << ", yaw=" << (int)(180.0*wp.position()[2]/M_PI) << "deg } -->";
    }
    std::cout << " (finished)" << std::endl;
  }

  std::cout << "\nFinish times for each agent:";
  for (const auto& [_, plan] : proposal)
  {
    const auto t = plan.get_waypoints().back().time().time_since_epoch();
    std::cout << " " << rmf_traffic::time::to_seconds(t);
  }
  std::cout << std::endl;
}

//==============================================================================
void evaluate(
  const Scenario& scenario,
  const std::vector<CentralizedNegotiation::Agent>& agents)
{
  for (const auto& a : agents)
  {
    std::cout << "Planning for ["
              << scenario.database->get_participant(a.id())->name()
              << "] to go from "
              << scenario.graph.get_waypoint(a.starts().front().waypoint()).name_or_index()
              << " to " << scenario.graph.get_waypoint(a.goal().waypoint()).name_or_index()
              << std::endl;
  }

  {
    std::cout << "\n -----\nFirst result found:\n";
    const auto begin = std::chrono::steady_clock::now();
    auto result = CentralizedNegotiation(scenario.database).solve(agents);
    const auto finish = std::chrono::steady_clock::now();
    print_result(scenario, result);
    std::cout << "time spent calculating: "
              << rmf_traffic::time::to_seconds(finish - begin) << std::endl;
    if (!result.proposal().has_value())
      return;
  }

  {
    std::cout << "\n -----\nBest result:\n";
    const auto begin = std::chrono::steady_clock::now();
    auto result = CentralizedNegotiation(scenario.database)
      .optimal().solve(agents);
    const auto finish = std::chrono::steady_clock::now();
    print_result(scenario, result);
    std::cout << "time spent calculating: "
              << rmf_traffic::time::to_seconds(finish - begin) << std::endl;
  }
}

//==============================================================================
void clockwise_rotation()
{
  Scenario scenario = make_scenario();
  const auto t = rmf_traffic::Time(rmf_traffic::Duration(0));

  std::vector<CentralizedNegotiation::Agent> agents;
  agents.push_back(
    {
      scenario.R0.id(),
      Start(t, scenario.graph.find_waypoint("Left")->index(), 0.0),
      Goal(scenario.graph.find_waypoint("Top")->index()),
      scenario.planner
    });

  agents.push_back(
    {
      scenario.R1.id(),
      Start(t, scenario.graph.find_waypoint("Top")->index(), -M_PI/2.0),
      Goal(scenario.graph.find_waypoint("Right")->index()),
      scenario.planner
    });

  agents.push_back(
    {
      scenario.R2.id(),
      Start(t, scenario.graph.find_waypoint("Right")->index(), M_PI),
      Goal(scenario.graph.find_waypoint("Left")->index()),
      scenario.planner
    });

  std::cout << "Clockwise Rotation:" << std::endl;
  evaluate(scenario, agents);
}

//==============================================================================
void challenging_problem()
{
  std::cout << "\n ============= \n" << std::endl;
  Scenario scenario = make_scenario();
  const auto t = rmf_traffic::Time(rmf_traffic::Duration(0));

  std::vector<CentralizedNegotiation::Agent> agents;
  agents.push_back(
    {
      scenario.R0.id(),
      Start(t, 0, 0.0),
      Goal(6),
      scenario.planner
    });

  agents.push_back(
    {
      scenario.R1.id(),
      Start(t, 1, 0.0),
      Goal(4),
      scenario.planner
    });

  agents.push_back(
    {
      scenario.R2.id(),
      Start(t, 7, M_PI),
      Goal(5),
      scenario.planner
    });

  std::cout << "Challenging Problem:" << std::endl;
  evaluate(scenario, agents);
  std::cout << "\nThis gives an illogical solution because the robots "
            << "\"disappear\" from the traffic schedule once they reach their "
            << "goals!" << std::endl;
}

//==============================================================================
int main()
{
  clockwise_rotation();

  // The next problem produces an illogical solution.
  // The current implementation of the traffic schedule system assumes that
  // robots will only park at locations where they will not interfere with the
  // traffic of other robots, therefore robots are removed from the traffic
  // schedule once they reach their goals. As a result, the negotiation allows
  // one robot to "pass through" another robot if that other robot has already
  // reached its goal location.
//  challenging_problem();
}
