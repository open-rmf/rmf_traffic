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

#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/geometry/Space.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test Space", "[space]")
{
  const auto circle1 = rmf_traffic::geometry::Circle(10.0);
  const auto final_circle1 = rmf_traffic::geometry::make_final_convex(circle1);
  const auto circle2 = rmf_traffic::geometry::Circle(10.0);
  const auto final_circle2 = rmf_traffic::geometry::make_final_convex(circle2);
  const auto circle3 = rmf_traffic::geometry::Circle(20.0);
  const auto final_circle3 = rmf_traffic::geometry::make_final_convex(circle3);

  Eigen::Isometry2d tf_identity1 = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d tf_identity2 = Eigen::Isometry2d::Identity();
  Eigen::Isometry2d tf_90 = Eigen::Isometry2d::Identity();
  tf_90.rotate(Eigen::Rotation2Dd(90.0*M_PI/180.0));

  GIVEN("Two spaces")
  {
    const auto space1 =
      rmf_traffic::geometry::Space{final_circle1, tf_identity1};
    const auto space2 =
      rmf_traffic::geometry::Space{final_circle2, tf_identity2};
    const auto space3 =
      rmf_traffic::geometry::Space{final_circle3, tf_identity1};
    const auto space4 =
      rmf_traffic::geometry::Space{final_circle1, tf_90};
    const auto space5 =
      rmf_traffic::geometry::Space{final_circle3, tf_90};
    WHEN("Both spaces are the same")
    {
      CHECK(space1 == space2);
    }

    WHEN("Spaces are different")
    {
      CHECK(space1 != space3);
      CHECK(space1 != space4);
      CHECK(space1 != space5);
    }
  }
}
