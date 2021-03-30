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
#ifndef RMF_TRAFFIC__RESERVATION__QUERY_HPP
#define RMF_TRAFFIC__RESERVATION__QUERY_HPP

#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>
#include <rmf_traffic/schedule/Participant.hpp>
#include <rmf_utils/impl_ptr.hpp>
#include <vector>

namespace rmf_traffic {
namespace reservations {
  class Query
  {
  public: 
    const std::vector<std::string> resources() const;

    static Query make_query(
      std::vector<std::string>& resources);
    class Implementation;

  private:
    rmf_utils::unique_impl_ptr<Implementation> _pimpl;
    Query();
  };
}
}

#endif