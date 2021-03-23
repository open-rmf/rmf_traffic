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

#ifndef RMF_TRAFFIC__RESERVATION__DATABASE_HPP
#define RMF_TRAFFIC__RESERVATION__DATABASE_HPP

#include <rmf_traffic/reservations/Writer.hpp>
#include <rmf_traffic/reservations/Viewer.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace reservations {
class Database: public Writer//, public Viewer
{
public:
  void make_reservation(
    std::vector<ReservationRequest> request, std::shared_ptr<Negotiator> nego);

  void set_duration(ReservationId id, rmf_traffic::Duration duration);

  void clear_duration(ReservationId id);
  
  void set_start_time(ReservationId id, rmf_traffic::Time time);

  void cancel(ReservationId id);

  //std::vector<Reservation> query(Query query);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};
}
}
#endif