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

#include <rmf_traffic/reservations/ReservationRequest.hpp>

namespace rmf_traffic {
namespace reservations {

class ReservationRequest::TimeRange::Implementation
{
public:
  std::optional<Time> _lower_bound;
  std::optional<Time> _upper_bound;
};


const std::optional<Time>
ReservationRequest::TimeRange::lower_bound() const
{
  return _pimpl->_lower_bound;
}
const std::optional<Time>
ReservationRequest::TimeRange::upper_bound() const
{
  return _pimpl->_upper_bound;
}

ReservationRequest::TimeRange
ReservationRequest::TimeRange::make_time_range(
  std::optional<Time> lower_bound,
  std::optional<Time> upper_bound)
{
  ReservationRequest::TimeRange tr;
  tr._pimpl->_lower_bound = lower_bound;
  tr._pimpl->_upper_bound = upper_bound;
  return tr;
}

ReservationRequest::TimeRange::TimeRange()
: _pimpl(rmf_utils::make_impl<Implementation>())
{

}

class ReservationRequest::Implementation
{
public:
  std::string _resource_name;
  std::optional<TimeRange> _start;
  std::optional<Duration> _duration;
  std::optional<Time> _finish;
};

//=============================================================================
const std::optional<ReservationRequest::TimeRange>
ReservationRequest::start_time() const
{
  return _pimpl->_start;
}

//=============================================================================
const std::optional<Duration> ReservationRequest::duration() const
{
  return _pimpl->_duration;
}

//=============================================================================
const std::optional<Time> ReservationRequest::finish_time() const
{
  return _pimpl->_finish;
}

//=============================================================================
const std::string ReservationRequest::resource_name() const
{
  return _pimpl->_resource_name;
}

//=============================================================================
bool ReservationRequest::is_indefinite() const
{
  return !_pimpl->_duration.has_value() && !_pimpl->_finish.has_value();
}

//=============================================================================
ReservationRequest ReservationRequest::make_request(
  std::string resource_name,
  std::optional<TimeRange> start,
  std::optional<Duration> duration,
  std::optional<Time> finish)
{
  ReservationRequest req;
  req._pimpl->_resource_name = resource_name;
  req._pimpl->_start = start;
  req._pimpl->_finish = finish;
  req._pimpl->_duration = duration;

  return req;
}

//=============================================================================
ReservationRequest::ReservationRequest()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
}

} // end namespace reservations
} // end namespace rmf_traffic
