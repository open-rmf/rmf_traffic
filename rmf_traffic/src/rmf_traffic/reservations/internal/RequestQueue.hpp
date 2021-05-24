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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTQUEUE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTQUEUE_HPP

#include <rmf_traffic/reservations/Database.hpp>
#include <unordered_map>

namespace rmf_traffic {
namespace reservations {

class RequestQueue
{
public:
  struct ReservationInfo {
    int priority;
    std::vector<ReservationRequest> request_options;
  };

  void enqueue_reservation(
    ParticipantId pid,
    RequestId req,
    int priority,
    std::vector<ReservationRequest>& reservation
  )
  {
    _reservation_info[pid].insert({
      req,
      {
        priority,
        std::move(reservation)
      }
    });
  }

  void erase_participant_requests(
    ParticipantId pid
  ){
    auto entry = _reservation_info.find(pid);
    if(entry == _reservation_info.end())
      return;
    _reservation_info.erase(entry);
  }

private:
  std::unordered_map<ParticipantId,
    std::unordered_map<RequestId, ReservationInfo>
    >  _reservation_info;
};

}
}
#endif