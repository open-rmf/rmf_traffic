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
#ifndef SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_VIEWIMPL 
#define SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_VIEWIMPL 
#include <rmf_traffic/reservations/Viewer.hpp>
#include "../detail/internal_bidirectional_iterator.hpp"
#include <map>
#include <unordered_map>

namespace rmf_traffic {
namespace reservations {

class Viewer::View::Implementation
{
public:
  using ReservationSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ReservationSchedules = std::unordered_map<std::string, ReservationSchedule>; 
public:
  ReservationSchedules schedule_holder;
  
  static Viewer::View make_view()
  {
    Viewer::View view;
    view._pimpl = rmf_utils::make_impl<Viewer::View::Implementation>();
    return view;
  }
};

class Viewer::View::IterImpl
{
  using ReservationSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ReservationSchedules = std::unordered_map<std::string, ReservationSchedule>; 
public:
  ReservationSchedules::const_iterator iter;
};

}
}
#endif