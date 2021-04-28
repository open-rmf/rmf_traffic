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

#include <rmf_utils/catch.hpp>
#include "../../../src/rmf_traffic/reservations/internal_State.hpp"

using namespace rmf_traffic::reservations;
using namespace std::chrono_literals;
SCENARIO("Given an empty schedule")
{
  CurrentScheduleState state;
  WHEN("adding a few reservations to a single node")
  {
    auto now = std::chrono::steady_clock::now();

    Reservation res1 = Reservation::make_reservation(
      now+10min,
      "node_1",
      0,
      {20min},
      std::nullopt
    );

    Reservation res2 = Reservation::make_reservation(
      now+40min,
      "node_1",
      0,
      {20min},
      std::nullopt
    );

    REQUIRE(state.add_reservation(res1));
    REQUIRE(state.add_reservation(res2));

    THEN("We have two reservations in the schedule")
    {
      auto sched = state.get_schedule("node_1");
      REQUIRE(sched.size() == 2);
      auto it = sched.begin();
      REQUIRE(it->first == res1.start_time());
      REQUIRE(it->second == res1);
      ++it;
      REQUIRE(it->first == res2.start_time());
      REQUIRE(it->second == res2);
    }

    THEN("We can retrieve the reservation by their ID")
    {
      auto res = state.get_reservation_by_id(res2.reservation_id());
      REQUIRE(res.has_value());
      REQUIRE(res.value() == res2);
    }
  }
  WHEN("Try to call get_schedule() on n on-existant entity")
  {
    THEN("throws an std::out_of_range")
    {
      REQUIRE_THROWS_AS(state.get_schedule("rubbish"), std::out_of_range);
    }
  }
}

SCENARIO("Given a schedule with two items")
{
  auto now = std::chrono::steady_clock::now();
  CurrentScheduleState state;

  Reservation res1 = Reservation::make_reservation(
    now+10min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  Reservation res2 = Reservation::make_reservation(
    now+40min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  state.add_reservation(res1);
  state.add_reservation(res2);

  WHEN("we try to update the time of the first reservation")
  {
    auto res1_updated = res1.propose_new_start_time(now+2h);
    REQUIRE(state.update_reservation(res1_updated));
    THEN("the reservation should have changed time")
    {
      auto res = state.get_reservation_by_id(res1.reservation_id());
      REQUIRE(res.has_value());
      REQUIRE(!(res.value() == res1));
      REQUIRE(res.value() == res1_updated);
      REQUIRE(state.get_schedule("node_1").size() == 2);
    }
  }
  WHEN("we try to overwrite the state")
  {
    auto res1_updated = res1.propose_new_start_time(now+2h);
    THEN("the adddition should fail.")
    {
      REQUIRE(!state.add_reservation(res1_updated));
      auto res = state.get_reservation_by_id(res1.reservation_id());
      REQUIRE(res.has_value());
      REQUIRE(res.value() == res1);
    }
  }
  WHEN("we try to cancel the state")
  {
    state.cancel_reservation(res1.reservation_id());
    THEN("the reservation should be removed")
    {
      REQUIRE(state.get_schedule("node_1").size() == 1);
      auto res = state.get_reservation_by_id(res1.reservation_id());
      REQUIRE(!res.has_value());
    }
  }
}

SCENARIO("Given a SchedulePatch with two items in its parent")
{
  auto now = std::chrono::steady_clock::now();
  auto state = std::make_shared<CurrentScheduleState>();

  Reservation res1 = Reservation::make_reservation(
    now+10min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  Reservation res2 = Reservation::make_reservation(
    now+40min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  state->add_reservation(res1);
  state->add_reservation(res2);

  SchedulePatch patch(state);

  WHEN("we try to update the time of the first reservation")
  {
    auto res1_updated = res1.propose_new_start_time(now+2h);
    REQUIRE(patch.update_reservation(res1_updated));
    THEN("the reservation should have changed time")
    {
      auto res = patch.get_reservation_by_id(res1.reservation_id());
      REQUIRE(res.has_value());
      REQUIRE(!(res.value() == res1));
      REQUIRE(res.value() == res1_updated);
      REQUIRE(patch.get_schedule("node_1").size() == 2);
    }
  }
  WHEN("we try to overwrite the state")
  {
    auto res1_updated = res1.propose_new_start_time(now+2h);
    THEN("the adddition should fail.")
    {
      REQUIRE(!patch.add_reservation(res1_updated));
      auto res = patch.get_reservation_by_id(res1.reservation_id());
      REQUIRE(res.has_value());
      REQUIRE(res.value() == res1);
    }
  }
  WHEN("we try to cancel the state")
  {
    patch.cancel_reservation(res1.reservation_id());
    THEN("the reservation should be removed")
    {
      REQUIRE(patch.get_schedule("node_1").size() == 1);
      auto res = patch.get_reservation_by_id(res1.reservation_id());
      REQUIRE(!res.has_value());
    }
  }
  WHEN("we try to add a reservation to the patch")
  {
    Reservation res3 = Reservation::make_reservation(
      now+40min,
      "node_1",
      0,
      {20min},
      std::nullopt
    );
    patch.add_reservation(res3);
  }
}