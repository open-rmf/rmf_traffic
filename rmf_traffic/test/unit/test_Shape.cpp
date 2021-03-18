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
#include <rmf_traffic/geometry/Shape.hpp>
#include <src/rmf_traffic/geometry/Box.hpp>
#include <src/rmf_traffic/geometry/SimplePolygon.hpp>

#include <rmf_utils/catch.hpp>

SCENARIO("Test shapes", "[shape]")
{
  // Tests for the equality operators.
  // TODO(Geoff): These tests are not exhaustive. Make them so.
  const auto box1 = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box1 = rmf_traffic::geometry::make_final_convex(box1);
  const auto box2 = rmf_traffic::geometry::Box(10.0, 1.0);
  const auto final_box2 = rmf_traffic::geometry::make_final_convex(box2);
  const auto box3 = rmf_traffic::geometry::Box(10.0, 10.0);
  const auto final_box3 = rmf_traffic::geometry::make_final_convex(box3);

  CHECK(*final_box1 == *final_box2);
  CHECK(*final_box1 != *final_box3);

  const auto circle1 = rmf_traffic::geometry::Circle(10.0);
  const auto final_circle1 = rmf_traffic::geometry::make_final_convex(circle1);
  const auto circle2 = rmf_traffic::geometry::Circle(10.0);
  const auto final_circle2 = rmf_traffic::geometry::make_final_convex(circle2);
  const auto circle3 = rmf_traffic::geometry::Circle(20.0);
  const auto final_circle3 = rmf_traffic::geometry::make_final_convex(circle3);

  CHECK(*final_circle1 == *final_circle2);
  CHECK(*final_circle1 != *final_circle3);

  Eigen::Vector2d edge1{10.0, 10.0};
  Eigen::Vector2d edge2{20.0, 20.0};
  Eigen::Vector2d edge3{30.0, 30.0};
  Eigen::Vector2d edge4{40.0, 40.0};
  Eigen::Vector2d edge5{50.0, 50.0};
  const auto simple_polygon1 =
    rmf_traffic::geometry::SimplePolygon({edge1, edge2, edge3});
  const auto final_simple_polygon1 =
    rmf_traffic::geometry::make_final(simple_polygon1);
  const auto simple_polygon2 =
    rmf_traffic::geometry::SimplePolygon({edge1, edge2, edge3});
  const auto final_simple_polygon2 =
    rmf_traffic::geometry::make_final(simple_polygon2);
  const auto simple_polygon3 =
    rmf_traffic::geometry::SimplePolygon({edge3, edge4, edge5});
  const auto final_simple_polygon3 =
    rmf_traffic::geometry::make_final(simple_polygon3);
  const auto simple_polygon4 =
    rmf_traffic::geometry::SimplePolygon({edge2, edge3, edge1});
  const auto final_simple_polygon4 =
    rmf_traffic::geometry::make_final(simple_polygon4);

  CHECK(*final_simple_polygon1 == *final_simple_polygon2);
  CHECK(*final_simple_polygon1 != *final_simple_polygon3);
  CHECK(*final_simple_polygon1 != *final_simple_polygon4);
}
