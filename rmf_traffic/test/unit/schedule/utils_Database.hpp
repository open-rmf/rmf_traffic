/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP
#define RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/Trajectory.hpp>

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>

#include <iostream>

#include <rmf_utils/catch.hpp>

inline void CHECK_EQUAL_TRAJECTORY(
  const rmf_traffic::Trajectory& t1,
  const rmf_traffic::Trajectory& t2)
{
  REQUIRE(t1.size() == t2.size());
  REQUIRE(t1.start_time());
  REQUIRE(t1.finish_time());
  REQUIRE(t2.start_time());
  REQUIRE(t2.finish_time());

  auto t1_it = t1.begin();
  auto t2_it = t2.begin();

  for (; t1_it != t1.end(); ++t1_it, ++t2_it)
  {
    REQUIRE((t1_it->position() - t2_it->position()).norm() == Approx(0.0).margin(
        1e-6));
    REQUIRE((t1_it->velocity() - t2_it->velocity()).norm() == Approx(0.0).margin(
        1e-6));
    REQUIRE((t1_it->time() - t2_it->time()).count() == Approx(0.0));
  }
}

inline void CHECK_TRAJECTORY_COUNT(
  const rmf_traffic::schedule::Viewer& d,
  const std::size_t expected_participant_num,
  const std::size_t expected_trajectory_num)
{
  const auto view = d.query(rmf_traffic::schedule::query_all());
  CHECK(view.size() == expected_trajectory_num);
  CHECK(d.participant_ids().size() == expected_participant_num);
}

inline std::vector<rmf_traffic::Trajectory> get_conflicting_trajectories(
  const rmf_traffic::schedule::Viewer::View& view,
  const rmf_traffic::Profile& p,
  const rmf_traffic::Trajectory& t)
{
  std::vector<rmf_traffic::Trajectory> collision_trajectories;
  for (const auto& v : view)
  {
    const auto& v_p = v.description.profile();
    const auto& v_t = v.route.trajectory();
    if (rmf_traffic::DetectConflict::between(v_p, v_t, p, t))
      collision_trajectories.push_back(v_t);
  }

  return collision_trajectories;
}

inline rmf_traffic::schedule::Writer::Input create_test_input(
  rmf_traffic::RouteId id, const rmf_traffic::Trajectory& t)
{
  return rmf_traffic::schedule::Writer::Input{
    {
      {id, std::make_shared<rmf_traffic::Route>("test_map", t)}
    }
  };
}


inline std::unordered_map<rmf_traffic::RouteId, rmf_traffic::ConstRoutePtr>
convert_itinerary(rmf_traffic::schedule::Writer::Input input)
{
  std::unordered_map<rmf_traffic::RouteId, rmf_traffic::ConstRoutePtr>
  itinerary;

  itinerary.reserve(input.size());

  for (const auto& item : input)
  {
    const auto result = itinerary.insert(std::make_pair(item.id, item.route));
    assert(result.second);
    (void)(result);
  }
  return itinerary;
}

#endif //RMF_TRAFFIC__TEST__UNIT__SCHEDULE__UTILS_TRAJECTORY_HPP
