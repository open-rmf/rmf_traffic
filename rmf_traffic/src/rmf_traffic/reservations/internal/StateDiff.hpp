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
#include "State.hpp"

namespace rmf_traffic {
namespace reservations {

/// Gives the difference between two states. *NOTE*: Assumes that 
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

  /// Calaculates the StateDiff required for one to go from State2 to State1.
  StateDiff(
    State& state1,
    State& state2
  )
  {
    // TODO(arjo): currently method is O(nlogn). It is possible to implement
    // in O(n) using a two pointer method, just trickier. Ideally we should 
    // do this.

    // Check for reservation shuffling/shifting and assignment
    for(auto [participant, requests]: state1._reservation_assignments)
    {
      for(auto [request, reservation_id1]: requests)
      {
        auto resource = state1._reservation_resources[reservation_id1];
        if(
          state2._reservation_assignments.count(participant) != 0
          && state2._reservation_assignments[participant].count(request) != 0)
        {
          auto reservation_id2 =
            state2._reservation_assignments[participant][request];
          auto time1 = state1._reservation_timings[reservation_id1];
          auto time2 = state2._reservation_timings[reservation_id2];
          auto res1 = state1._resource_schedules[resource].find(time1)->second;
          auto res2 = state2._resource_schedules[resource].find(time2)->second;

          if(res1 != res2)
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
          auto res1 = state1._resource_schedules[resource].find(time1).second;
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
  }
private:
  std::vector<Difference>  _differences;
};

}
}
#endif