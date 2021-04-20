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

//#include <rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/schedule/Query.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test Query API", "[query]")
{
  using namespace std::chrono_literals;

  rmf_traffic::schedule::Query query = rmf_traffic::schedule::make_query({});

  auto now = std::chrono::steady_clock::now();

  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  REQUIRE(query.spacetime().regions() != nullptr);

  query.spacetime().regions()->push_back(
    rmf_traffic::Region{"test_map", now, now+10s, {}});

  REQUIRE(query.spacetime().regions()->begin()
    != query.spacetime().regions()->end());

  auto& region = *query.spacetime().regions()->begin();

  REQUIRE(region.get_lower_time_bound() != nullptr);
  CHECK(*region.get_lower_time_bound() == now);
  REQUIRE(region.get_upper_time_bound() != nullptr);
  CHECK(*region.get_upper_time_bound() == now+10s);

  region.push_back(rmf_traffic::geometry::Space{final_box, tf});

  tf.rotate(Eigen::Rotation2Dd(90.0*M_PI/180.0));
  region.push_back(rmf_traffic::geometry::Space{final_box, tf});
  tf.rotate(Eigen::Rotation2Dd(180.0*M_PI/180.0));
  region.push_back(rmf_traffic::geometry::Space{final_box, tf});

  std::size_t regions = 0;
  std::size_t spaces = 0;
  for (const auto& region : *query.spacetime().regions())
  {
    ++regions;
    for ([[maybe_unused]] const auto& space : region)
    {
      ++spaces;
    }
  }

  CHECK(regions == 1);
  CHECK(spaces == 3);

  // Tests for the equality operators.
  GIVEN("Two queries")
  {
    auto now_plus_30s = now + 30s;
    auto now_plus_1min = now + 1min;
    // TODO(Geoff): These tests are not exhaustive. Make them so.
    WHEN("Both queries are empty")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});

      CHECK(query1 == query2);
    }

    WHEN("Both queries have the same start and end times")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_1min);
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_1min);

      CHECK(query1 == query2);
    }

    WHEN("The queries have different start times")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_1min);
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({}, &now_plus_30s, &now_plus_1min);

      CHECK(query1 != query2);
    }

    WHEN("The queries have different finish times")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_30s);
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_1min);

      CHECK(query1 != query2);
    }

    WHEN("One query has a spacetime query")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.spacetime().regions()->push_back(
        rmf_traffic::Region{"test_map", now, now+10s, {}});

      CHECK(query1 != query2);
    }

    WHEN("Both queries have the same spacetime query")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.spacetime().regions()->push_back(
        rmf_traffic::Region{"test_map", now, now+10s, {}});
      query2.spacetime().regions()->push_back(
        rmf_traffic::Region{"test_map", now, now+10s, {}});

      CHECK(query1 == query2);
    }

    WHEN("Both queries are equal timespans for all maps, "
      "but one has a map explicitly added")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({"test_map"}, &now, &now_plus_30s);
      query1.spacetime().timespan()->all_maps(true);

      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({}, &now, &now_plus_30s);
      query2.spacetime().timespan()->all_maps(true);

      CHECK(query1 == query2);
    }

    WHEN("Both queries have different spacetime queries")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.spacetime().regions()->push_back(
        rmf_traffic::Region{"test_map", now, now+10s, {}});
      query2.spacetime().regions()->push_back(
        rmf_traffic::Region{"another_map", now, now+10s, {}});

      CHECK(query1 != query2);
    }

    WHEN("One query has a participant query")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({1, 2, 3});

      CHECK(query1 != query2);
    }

    WHEN("Both queries have the same participant query")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({1, 2, 3});
      query2.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({1, 2, 3});

      CHECK(query1 == query2);
    }

    WHEN("Both queries have different participant queries")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({1, 2, 3});
      query2.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({2, 3, 4});

      CHECK(query1 != query2);

      rmf_traffic::schedule::Query query3 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query4 =
        rmf_traffic::schedule::make_query({});
      query3.participants() =
        rmf_traffic::schedule::Query::Participants::make_all_except({1, 2, 3});
      query4.participants() =
        rmf_traffic::schedule::Query::Participants::make_all_except({2, 3, 4});

      CHECK(query3 != query4);
    }

    WHEN("Both queries have different participant query modes")
    {
      rmf_traffic::schedule::Query query1 =
        rmf_traffic::schedule::make_query({});
      rmf_traffic::schedule::Query query2 =
        rmf_traffic::schedule::make_query({});
      query1.participants() =
        rmf_traffic::schedule::Query::Participants::make_only({1, 2, 3});
      query2.participants() =
        rmf_traffic::schedule::Query::Participants::make_all_except({1, 2, 3});

      CHECK(query1 != query2);
    }
  }

  // TODO(MXG): Write tests for every function to confirm that the
  // Query API works as intended
}
