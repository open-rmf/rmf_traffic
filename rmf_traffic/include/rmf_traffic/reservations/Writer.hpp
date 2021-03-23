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

#ifndef RMF_TRAFFIC__RESERVATION__WRITER_HPP
#define RMF_TRAFFIC__RESERVATION__WRITER_HPP

#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>
#include <rmf_traffic/reservations/Negotiator.hpp>

namespace rmf_traffic {
namespace reservations {
class Writer
{
public: 
  virtual void make_reservation(
    std::vector<ReservationRequest> request, std::shared_ptr<Negotiator> nego) = 0;
  
  virtual void set_duration(ReservationId id, rmf_traffic::Duration duration) = 0;

  virtual void clear_duration(ReservationId id) = 0;

  virtual void set_start_time(ReservationId id, rmf_traffic::Time time) = 0;
  
  virtual void cancel(ReservationId id) = 0;
};
}
}

#endif