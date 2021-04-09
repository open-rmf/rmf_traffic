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
#include <random>

SCENARIO("Test LaneClosure class functions")
{
  rmf_traffic::agv::LaneClosure closure;
  const std::size_t lane_num = 100;

//  WHEN("Default LaneClosure")
  {
//    THEN("All lanes are open")
    {
      for (std::size_t i = 0; i < lane_num; ++i)
      {
        CHECK(closure.is_open(i));
        CHECK_FALSE(closure.is_closed(i));
      }
    }
  }

//  WHEN("Lane 73 is closed")
  {
    closure.close(73);
    CHECK(closure.hash() != 0);

//    THEN("All lanes but 73 are open")
    {
      for (std::size_t i = 0; i < lane_num; ++i)
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

//  WHEN("Lane 73 is reopened")
  {
    closure.open(73);
    CHECK(closure.hash() == 0);

//    THEN("All lanes are open")
    {
      for (std::size_t i = 0; i < lane_num; ++i)
      {
        CHECK(closure.is_open(i));
        CHECK_FALSE(closure.is_closed(i));
      }
    }
  }

  std::unordered_set<rmf_traffic::agv::LaneClosure> unique_closures;

  closure.close(0);
  CHECK(closure.hash() == 1);
  CHECK_FALSE(closure.is_open(0));
  CHECK(closure.is_closed(0));

  unique_closures.insert(closure);

  closure.close(64);
  {
    // NOTE(MXG): This is an implementation-specific expectation which can be
    // relaxed if the implementation of the hash function is ever changed in the
    // future.
    CHECK(closure.hash() == 1);
  }
  CHECK_FALSE(closure.is_open(0));
  CHECK(closure.is_closed(0));
  CHECK_FALSE(closure.is_open(64));
  CHECK(closure.is_closed(64));

  // Since the value of the closure has changed, it should occupy a new spot in
  // the set of unique closures, even though its hash has not changed.
  unique_closures.insert(closure);
  CHECK(unique_closures.size() == 2);

  closure.open(64);
  unique_closures.insert(closure);
  // We should just be inserting the same closure value that we did originally,
  // so there should still only be 2 closures in the unique set.
  CHECK(unique_closures.size() == 2);
}

SCENARIO("Fuzz test of LaneClosure", "[debug]")
{
  const std::size_t num_lanes = 150;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<std::size_t> dist(0, num_lanes);

  std::unordered_set<std::size_t> closed_lanes;
  const std::size_t num_closed_lanes = 10;
  while (closed_lanes.size() < num_closed_lanes)
  {
    closed_lanes.insert(dist(gen));
  }

  rmf_traffic::agv::LaneClosure closure;
  std::unordered_set<rmf_traffic::agv::LaneClosure> unique_closures;
  for (const auto& v : closed_lanes)
  {
    unique_closures.insert(closure);
    closure.close(v);
  }

  // There should be one unique closure in this set for each lane that we have
  // closed.
  CHECK(unique_closures.size() == num_closed_lanes);

  while (closed_lanes.size() > 0)
  {
    for (std::size_t i = 0; i < num_lanes; ++i)
    {
      if (closed_lanes.count(i))
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

    const auto pop = closed_lanes.begin();
    closure.open(*pop);
    closed_lanes.erase(pop);
  }

  for (std::size_t i = 0; i < num_lanes; ++i)
  {
    CHECK(closure.is_open(i));
    CHECK_FALSE(closure.is_closed(i));
  }
}
