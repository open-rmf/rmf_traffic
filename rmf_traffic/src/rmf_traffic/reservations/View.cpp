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

#include "internal_ViewImpl.hpp"

namespace rmf_traffic {
namespace reservations {

class Viewer::View::IterImpl
{
  using ReservationSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ReservationSchedules = std::unordered_map<std::string, ReservationSchedule>; 
public:
  ReservationSchedules::const_iterator iter;
};

Viewer::View::const_iterator Viewer::View::begin() const
{
  return Viewer::View::const_iterator{IterImpl{_pimpl->schedule_holder.begin()}};
}

Viewer::View::const_iterator Viewer::View::end() const
{
  return Viewer::View::const_iterator{IterImpl{_pimpl->schedule_holder.end()}};
}

std::size_t Viewer::View::size() const
{
  return _pimpl->schedule_holder.size();
}

}
}