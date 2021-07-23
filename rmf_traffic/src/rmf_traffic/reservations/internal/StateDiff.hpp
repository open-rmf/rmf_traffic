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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATEDIFF_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATEDIFF_HPP

#include <unordered_map>
#include <unordered_set>
#include <queue>
#include "State.hpp"

namespace rmf_traffic {
namespace reservations {

//==============================================================================
/// Gives the difference between two states. *NOTE*: Assumes that both the
/// states have the same
class StateDiff
{
public:

  enum DifferenceType
  {
    SHIFT_RESERVATION,
    ASSIGN_RESERVATION,
    UNASSIGN_RESERVATION
  };

  struct Difference
  {
    DifferenceType diff_type;
    ParticipantId participant_id;
    RequestId request_id;
    std::optional<Reservation> reservation;
  };

  //============================================================================
  /// Calculates the StateDiff required for one to go from State2 to State1.
  StateDiff(
    State& state1,
    State& state2
  )
  {
    // TODO(arjo): currently method is O(nlogn). It is possible to implement
    // in O(n) using a two pointer method, just trickier. Ideally we should
    // do this.

    // Check for reservation shuffling/shifting and assignment
    for (auto [participant, requests]: state1._reservation_assignments)
    {
      for (auto [request, reservation_id1]: requests)
      {
        auto resource = state1._reservation_resources[reservation_id1];
        if (
          state2._reservation_assignments.count(participant) != 0
          && state2._reservation_assignments[participant].count(request) != 0)
        {
          auto reservation_id2 =
            state2._reservation_assignments[participant][request];
          auto time1 = state1._reservation_timings[reservation_id1];
          auto time2 = state2._reservation_timings[reservation_id2];
          auto res1 = state1._resource_schedules[resource].find(time1)->second;
          auto res2 = state2._resource_schedules[resource].find(time2)->second;

          if (res1 != res2)
          {
            // Request was shifted from state 2
            Difference diff
            {
              DifferenceType::SHIFT_RESERVATION,
              participant,
              request,
              {res1}
            };
            _differences.push_back(diff);
          }
        }
        else
        {
          // Request was not there in state2 therefore we assign it.
          auto time1 = state1._reservation_timings[reservation_id1];
          auto res1 = state1._resource_schedules[resource].find(time1)->second;
          Difference diff
          {
            DifferenceType::ASSIGN_RESERVATION,
            participant,
            request,
            {res1}
          };
          _differences.push_back(diff);
        }

      }
    }

    //Check for unassigning
    for (auto [participant, request]: state1.unassigned())
    {
      if (state2._reservation_assignments.count(participant) > 0
        && state2._reservation_assignments[participant].count(request) > 0)
      {
        _differences.push_back(
          {
            UNASSIGN_RESERVATION,
            participant,
            request,
            std::nullopt
          }
        );
      }
    }

    //_differences = toposort(state2);
  }

  //============================================================================
  /// Return the differences.
  std::vector<Difference> differences() const
  {
    return _differences;
  }
private:
  //============================================================================
  /// Use something akin to Kahn's algorithm to perform a topological sort of
  /// the shifts. This ensures that during the rollout phase, if there is a
  /// communications failure the system will remain in a usable state.
  std::vector<Difference> toposort(State& state)
  {
    std::vector<Difference> result;
    std::queue<std::size_t> unvisited;
    State curr_state(state);
    //Unassignments can go first, since they will never create a conflict.
    for (std::size_t i = 0; i < _differences.size(); i++)
    {
      if (_differences[i].diff_type == DifferenceType::UNASSIGN_RESERVATION)
      {
        result.push_back(_differences[i]);
        curr_state = curr_state.unassign_reservation(
          _differences[i].participant_id,
          _differences[i].request_id);
      }
      else
      {
        unvisited.push(i);
      }
    }

    while (!unvisited.empty())
    {
      auto index = unvisited.front();
      unvisited.pop();
      auto diff = _differences[index];
      auto reservation = diff.reservation;

      if (!curr_state.check_if_conflicts(reservation.value()))
      {
        if (diff.diff_type == DifferenceType::SHIFT_RESERVATION)
          curr_state = curr_state.unassign_reservation(
            diff.participant_id,
            diff.request_id
          );

        curr_state.assign_reservation(
          diff.participant_id,
          diff.request_id,
          reservation.value()
        );
      }
      else
      {
        unvisited.push(index);
      }
    }

    return result;
  }

  std::vector<Difference> _differences;
};

} //end namespace reservations
} //end namespace rmf_traffic
#endif
