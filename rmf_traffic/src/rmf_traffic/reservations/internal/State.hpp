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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP

#include <rmf_traffic/reservations/Database.hpp>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include "RequestQueue.hpp"

namespace rmf_traffic {
namespace reservations {
class State
{
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };
public:
  /// Adds a reservation to the state
  /// Note: Assumes that reservation is valid.
  State add_reservation(
    ParticipantId pid,
    RequestId reqid,
    Reservation res)
  {

  }

  State remove_request(
    ParticipantId pid,
    RequestId reqid)
  {

  }

  State remove_reservation(ReservationId res)
  {

  }

  std::optional<State> delay_reservations_after(ReservationId id, Duration dur)
  {

  }

  std::optional<State>
    bring_forward_reservations_before(ReservationId id, Duration dur)
  {

  }

private:
  std::unordered_map<
    std::pair<ParticipantId, RequestId>, ReservationId, pair_hash> _assignments;

  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules;

  std::unordered_set<std::pair<ParticipantId, RequestId>, pair_hash> _unassigned;
};

}
}
#endif