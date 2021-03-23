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

#ifndef RMF_TRAFFIC__RESERVATIONS_HPP
#define RMF_TRAFFIC__RESERVATIONS_HPP

#include <optional>
#include <vector>
#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_utils/impl_ptr.hpp>



namespace rmf_traffic {
namespace reservations {

using ReservationId = uint64_t;
//==============================================================================  
class Reservation
{
public:
  const rmf_traffic::Time start_time() const;

  const std::optional<rmf_traffic::Duration> duration() const;

  const std::string resource_name() const;
  
  ReservationId reservation_id() const;

  schedule::ParticipantId participant_id() const;

  bool operator< (const Reservation& other) const;

  static Reservation make_reservation(
    rmf_traffic::Time start_time,
    std::string resource_name,
    schedule::ParticipantId pid,
    std::optional<rmf_traffic::Duration> duration);
  
  class Implementation;
private:
  Reservation();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};
} // end namespace reservations
} // end namespace rmf_traffic
#endif
