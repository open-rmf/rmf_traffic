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

///=============================================================================
// Writer class 
class Writer
{
public:

  ///===========================================================================
  /// Registers a participant
  /// \param[in] id the Participant's ID. If a participant registers with an
  /// already pre-existing id then the id should get overwritten.
  /// (Warning: currently overwrites are not threadsafe and should be avoided.
  /// This needs to be fixed before merging)
  /// \param[in] participant a shared pointer pointing to the participant
  /// handler
  virtual void register_participant(
    ParticipantId id,
    std::shared_ptr<Participant> participant) = 0;

  ///===========================================================================
  /// Removes a participant.
  /// \param[in] id the participant's id
  virtual void unregister_participant(
    ParticipantId id
  ) = 0;

  ///===========================================================================
  /// Request a reservation.
  /// \param[in] id the participant requesting the reservation
  /// \param[in] req the request id. This should be unique for each request and
  /// needs to be tracked by the client. (Warning to be fixed before merge:
  /// If a request is duplicated there is undefined behaviour. This needs to
  /// change to log an error or throw an exception...)
  /// \param[in] request_options alternatives which can service this request. 
  /// For instance, if we would like to reserve a charger we can send in a 
  /// ReservationRequest for all the chargers. Suppose Charger1 is nearest and
  /// Charger4 is farthest and we prefer near chargers then we would send in a
  /// list of requests like so {Charger1, Charger2, Charger3, Charger4}. The
  /// system should assign Charger1 lowest cost.
  /// \param[in] priority The priority to give this reservation. Higher priority
  /// requests will be given preference when solving/optimizing for the best
  /// solution.
  virtual void request_reservation(
    ParticipantId id,
    RequestId req,
    std::vector<ReservationRequest>& request_options,
    int priority = 0
  ) = 0;

  /// Cancels a request.
  /// \param[in] id - the participant cancelling the request
  /// \param[req] req -the request id
  virtual void cancel_request(ParticipantId id, RequestId req) = 0;
};
}
}

#endif