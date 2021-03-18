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

#include <src/rmf_traffic/geometry/Box.hpp>

#include <rmf_traffic/Region.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test Region API", "[region]")
{
  using namespace std::chrono_literals;

  auto now = std::chrono::steady_clock::now();
  const auto box = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box = rmf_traffic::geometry::make_final_convex(box);
  Eigen::Isometry2d tf = Eigen::Isometry2d::Identity();

  // Tests for the equality operators.
  // TODO(Geoff): These tests are not exhaustive. Make them so.
  GIVEN("Two regions")
  {
    auto now_plus_30s = now + 30s;
    auto now_plus_1min = now + 1min;

    WHEN("Both regions are empty")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      CHECK(region1 == region2);
    }

    WHEN("Regions have different maps")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map2", {});

      CHECK(region1 != region2);
    }

    WHEN("Regions have the same lower bounds")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.set_lower_time_bound(now_plus_30s);
      CHECK(region1 != region2);

      region2.set_lower_time_bound(now_plus_30s);
      CHECK(region1 == region2);
    }

    WHEN("Regions have different lower bounds")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.set_lower_time_bound(now_plus_30s);
      CHECK(region1 != region2);

      region2.set_lower_time_bound(now_plus_1min);
      CHECK(region1 != region2);
    }

    WHEN("Regions have the same upper bounds")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.set_upper_time_bound(now_plus_30s);
      CHECK(region1 != region2);

      region2.set_upper_time_bound(now_plus_30s);
      CHECK(region1 == region2);
    }

    WHEN("Regions have different upper bounds")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.set_upper_time_bound(now_plus_30s);
      CHECK(region1 != region2);

      region2.set_upper_time_bound(now_plus_1min);
      CHECK(region1 != region2);
    }

    WHEN("Regions have the same spaces")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.push_back(rmf_traffic::geometry::Space{final_box, tf});
      region2.push_back(rmf_traffic::geometry::Space{final_box, tf});

      CHECK(region1 == region2);
    }

    WHEN("Regions have different spaces")
    {
      rmf_traffic::Region region1("map1", {});
      rmf_traffic::Region region2("map1", {});

      region1.push_back(rmf_traffic::geometry::Space{final_box, tf});
      CHECK(region1 != region2);

      tf.rotate(Eigen::Rotation2Dd(90.0*M_PI/180.0));
      region2.push_back(rmf_traffic::geometry::Space{final_box, tf});
      CHECK(region1 != region2);
    }
  }
}
