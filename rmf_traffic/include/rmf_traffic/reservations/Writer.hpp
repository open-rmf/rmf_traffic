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

#ifndef RMF_TRAFFIC__WRITER_HPP
#define RMF_TRAFFIC__WRITER_HPP

#include <rmf_traffic/reservations/Participant.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>

namespace rmf_traffic {
namespace reservations {

class Writer
{
public:
  virtual void register_participant(
    ParticipantId id,
    std::shared_ptr<Participant> participant) = 0;

  virtual void unregister_participant(
    ParticipantId id
  ) = 0;

  virtual void request_reservation(
    ParticipantId id,
    RequestId req,
    std::vector<ReservationRequest>& request_options,
    int priority = 0
  ) = 0;

  virtual void cancel_request(ParticipantId id, RequestId req) = 0;
};
}
}

#endif