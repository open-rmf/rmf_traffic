/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include "../../../../src/rmf_traffic/reservations/internal/State.hpp"
#include "../../../../src/rmf_traffic/reservations/internal/StateDiff.hpp"
#include "../../../../src/rmf_traffic/reservations/internal/Heuristic.hpp"
#include "../../../../src/rmf_traffic/reservations/internal/Optimizer.hpp"

#include <rmf_utils/catch.hpp>

using namespace rmf_traffic;
using namespace rmf_traffic::reservations;
using namespace std::chrono_literals;

SCENARIO("A few reservations in a state")
{
  GIVEN("A state with one reservation")
  {
    auto queue = std::make_shared<RequestStore>();
    State start_state(queue);
    auto now = std::chrono::steady_clock::now();
    now -= now.time_since_epoch();
    auto request1_alt1 = ReservationRequest::make_request(
      "table_at_timbre",
      ReservationRequest::TimeRange::make_time_range(
        now,
        now+5s
      ),
      {10s}
    );

    auto request1 = std::vector{request1_alt1};
    start_state = start_state.add_request(0, 0, 1, request1);

    std::vector<State> child_states;
    for (auto child_state: start_state)
    {
      child_states.push_back(child_state);
    }

    REQUIRE(child_states.size() == 1);
    auto allocated_state = child_states[0].add_request(1, 0, 20, request1);

    WHEN("We have a conflicting request with higher priority")
    {
      auto heuristic = std::make_shared<PriorityBasedScorer>();
      GreedyBestFirstSearchOptimizer opt(heuristic);
      auto solutions = opt.optimize(allocated_state);
      std::vector<State> result;
      while (auto solution = solutions.next_solution())
      {
        result.push_back(solution.value());
      }

      REQUIRE(result.size() == 2);

      THEN("The request with a higher priority will be serviced first")
      {
        StateDiff diff(result[0], allocated_state);
        auto steps = diff.differences();
        for (auto step: steps)
        {
          if (step.diff_type == StateDiff::DifferenceType::ASSIGN_RESERVATION)
          {
            REQUIRE(step.participant_id == 1);
          }
          else if (step.diff_type ==
            StateDiff::DifferenceType::UNASSIGN_RESERVATION)
          {
            REQUIRE(step.participant_id == 0);
          }
        }
        REQUIRE(steps.size() == 2);
      }
    }
  }

  GIVEN("An empty state with two requests pending")
  {
    auto queue = std::make_shared<RequestStore>();
    State start_state(queue);
    auto now = std::chrono::steady_clock::now();
    now -= now.time_since_epoch();
    auto request1_alt1 = ReservationRequest::make_request(
      "table_at_timbre",
      ReservationRequest::TimeRange::make_time_range(
        now,
        now+10s
      ),
      {10s}
    );
    auto request1_alt2 = ReservationRequest::make_request(
      "table_at_koufu",
      ReservationRequest::TimeRange::make_time_range(
        now,
        now+10s
      ),
      {10s}
    );

    auto request1 = std::vector{request1_alt1, request1_alt2};
    start_state = start_state.add_request(0, 0, 1, request1);

    auto request2_alt1 = ReservationRequest::make_request(
      "table_at_timbre",
      ReservationRequest::TimeRange::make_time_range(
        now,
        now+10s
      ),
      {10s}
    );
    auto request2 = std::vector{request2_alt1};
    start_state = start_state.add_request(1, 0, 1, request2);

    WHEN("We request the next possible states")
    {
      std::vector<State> child_states;

      for (auto next_state: start_state)
      {
        child_states.push_back(next_state);
      }
      THEN("There are three possible allocations")
      {
        REQUIRE(child_states.size() == 3);
      }
      THEN("The state difference between all child states is 1 add element")
      {
        for (auto child_state: child_states)
        {
          StateDiff diff(child_state, start_state);
          auto steps = diff.differences();
          REQUIRE(steps.size() == 1);
          REQUIRE(steps[0].diff_type
            == StateDiff::DifferenceType::ASSIGN_RESERVATION);
        }
      }
    }

    WHEN("We attempt to solve")
    {
      auto heuristic = std::make_shared<PriorityBasedScorer>();
      GreedyBestFirstSearchOptimizer opt(heuristic);
      auto solutions = opt.optimize(start_state);
      std::vector<float> scores;
      while (auto solution = solutions.next_solution())
      {
        scores.push_back(heuristic->score(solution.value()));
      }

      THEN("The scores come back in ascending order")
      {
        for(std::size_t i = 1; i < scores.size(); ++i)
        {
          REQUIRE(scores[i-1] <= scores[i]);
        }
      }
    }

    WHEN("Checking for conflict against another non-conflicting reservation")
    {
      Reservation res = Reservation::make_reservation(
        now,
        "Hawaii",
        {},
        {}
      );

      THEN("There should be no conflict")
      {
        REQUIRE(start_state.check_if_conflicts(res) == false);
      }
    }
  }
}

// TODO: Increase test cases here to cover all edge cases
SCENARIO("Given a single item allocated in a state")
{
  auto queue = std::make_shared<RequestStore>();
  State start_state(queue);
  auto now = std::chrono::steady_clock::now();
  now -= now.time_since_epoch();
  auto request1_alt1 = ReservationRequest::make_request(
    "table_at_timbre",
    ReservationRequest::TimeRange::make_time_range(
      now,
      now+10s
    ),
    {10s}
  );

  auto request1 = std::vector{request1_alt1};
  start_state = start_state.add_request(0, 0, 1, request1);

  std::vector<State> next_state;
  for (auto state: start_state)
  {
    next_state.push_back(state);
  }

  WHEN("Checking for overlapping indefinite reservation")
  {
    Reservation res = Reservation::make_reservation(
      now,
      "table_at_timbre",
      {},
      {}
    );

    THEN("check_if_conflicts returns true")
    {
      REQUIRE(next_state[0].check_if_conflicts(res));
    }
  }

  WHEN("Checking for overlapping indefinite reservation")
  {
    Reservation res = Reservation::make_reservation(
      now,
      "table_at_koufu",
      {},
      {}
    );

    THEN("check_if_conflicts returns false")
    {
      REQUIRE(!next_state[0].check_if_conflicts(res));
    }
  }
}
