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

#ifndef RMF_TRAFFIC__PARTICIPANT_HPP
#define RMF_TRAFFIC__PARTICIPANT_HPP

#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>
namespace rmf_traffic {
namespace reservations {

using ParticipantId = uint64_t;

///=============================================================================
/// \brief This class is used to handle the asynchronous callbacks made by the
/// database. In order to make a reservation you will need to inherit this class
class Participant
{
public:
  ///===========================================================================
  /// \brief This method is called whenever a change is proposed by the
  /// reservation system to the participant. It is also called when you first
  /// make a request.
  /// \param[in] id is the RequestId of the request that will be impacted.
  /// \param[in] res is the Reservation that is being proposed by the
  /// reservation system.
  /// \return true if the request is acceptable, false otherwise.
  virtual bool request_proposal(
    RequestId id,
    Reservation& res
  ) = 0;

  ///===========================================================================
  /// \brief This is called when the solution to a request is confirmed. It will
  /// only be called if all the participants that are impacted by the set of
  /// changes needed have approved of it. Note: once this has been called it is
  /// expected that the participant will comply.
  /// \param[in] id the request id.
  /// \param[in] res the response.
  /// \return true if the request has been successfully confirmed, false
  /// otherwise
  virtual bool request_confirmed(
    RequestId id,
    Reservation& res
  ) = 0;

  ///===========================================================================
  /// \brief This method is called whenever a change is proposed by the
  /// reservation system to the participant to cancel the request made by it.
  /// The request will still remain in the database, just that it will be
  /// serviced at some point later if possible.
  /// \param[in] id is the RequestId of the request that will be impacted.
  /// \return true if the you agree to allow your request to be cancelled, false
  /// otherwise.
  virtual bool unassign_request_confirmed(
    RequestId id
  ) = 0;

  ///===========================================================================
  /// \brief This method is called whenever a cancellation has been allowed.
  /// \param[in] id is the RequestId of the request that will be impacted.
  /// \return true if the request is acceptable, false otherwise.
  virtual bool unassign_request_proposal(
    RequestId id
  ) = 0;
};

}
}
#endif