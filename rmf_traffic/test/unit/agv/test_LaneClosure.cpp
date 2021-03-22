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

#include <rmf_traffic/agv/LaneClosure.hpp>

#include <rmf_utils/catch.hpp>

#include <unordered_set>

SCENARIO("Test LaneClosure class")
{
  rmf_traffic::agv::LaneClosure closure;
  const std::size_t lane_num = 100;

//  WHEN("Default LaneClosure")
  {
//    THEN("All lanes are open")
    {
      for (std::size_t i=0; i < lane_num; ++i)
      {
        CHECK(closure.is_open(i));
        CHECK_FALSE(closure.is_closed(i));
      }
    }
  }

//  WHEN("Lane 73 is closed")
  {
    closure.close(73);
    CHECK(closure.hash() == (1 << 73));

//    THEN("All lanes but 73 are open")
    {
      for (std::size_t i=0; i < lane_num; ++i)
      {
        if (73 == i)
        {
          CHECK_FALSE(closure.is_open(i));
          CHECK(closure.is_closed(i));
        }
        else
        {
          CHECK(closure.is_open(i));
          CHECK_FALSE(closure.is_closed(i));
        }
      }
    }
  }
}
