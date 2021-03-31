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

#include "internal_DatabasImpl.hpp"
#include "internal_ViewImpl.hpp"
namespace rmf_traffic {
namespace reservations {

void Database::make_reservation(
  std::vector<ReservationRequest> request,
  std::shared_ptr<Negotiator> nego)
{
  _pimpl->make_reservation(request, nego);
}

void Database::set_duration(ReservationId id, rmf_traffic::Duration duration)
{

}

void Database::clear_duration(ReservationId id)
{

}

void Database::set_start_time(ReservationId id, rmf_traffic::Time time)
{

}

void Database::cancel(ReservationId id)
{

}

Viewer::View Database::query(Query& query)
{
  auto resources = query.resources();
  auto view = Viewer::View::Implementation::make_view();
  for(auto resource: resources)
  {
    view._pimpl->schedule_holder[resource] = _pimpl->_resource_schedules[resource];
  }
  return view;
}

//==============================================================================
Database::Database()
  : _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

}
}