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

#include <rmf_traffic/reservations/Reservation.hpp>
#include <atomic>

namespace rmf_traffic {
namespace reservations {

//=============================================================================
class Reservation::Implementation
{
public:
  ReservationId _res_id;
  std::string _resource_name;
  schedule::ParticipantId _pid;
  rmf_traffic::Time _start_time;
  std::optional<rmf_traffic::Duration>  _duration;
  std::optional<rmf_traffic::Time>  _finish_time;
};

//=============================================================================
const rmf_traffic::Time Reservation::start_time() const
{
  return _pimpl->_start_time;
}

//=============================================================================
const std::optional<rmf_traffic::Duration> Reservation::duration() const
{
  return _pimpl->_duration;
}

//=============================================================================
const std::optional<rmf_traffic::Time> Reservation::finish_time() const
{
  return _pimpl->_finish_time;
}

//=============================================================================
void Reservation::set_actual_finish_time(rmf_traffic::Time dur)
{
  if(dur < _pimpl->_start_time)
    _pimpl->_start_time = dur;

  _pimpl->_finish_time = dur;
  _pimpl->_duration.emplace(*_pimpl->_finish_time - _pimpl->_start_time);
}

//=============================================================================
Reservation Reservation::propose_new_finish_time(rmf_traffic::Time time)
{
  Reservation res(*this);
  res.set_actual_finish_time(time);
  return res;
}

//=============================================================================
Reservation Reservation::propose_new_start_time(rmf_traffic::Time start_time)
{
  Reservation res(*this);
  res._pimpl->_start_time = start_time;
  return res;
}

//=============================================================================
const std::optional<rmf_traffic::Time> Reservation::actual_finish_time() const
{
  if(is_indefinite()) return std::nullopt;

  if(_pimpl->_finish_time.has_value() && _pimpl->_duration.has_value())
  {
    return {std::max(_pimpl->_start_time + *_pimpl->_duration, 
      *_pimpl->_finish_time)};
  }
  else if(_pimpl->_finish_time.has_value())
  {
    return {std::max(_pimpl->_start_time, *_pimpl->_finish_time)};
  }
  else
  {
    return _pimpl->_start_time + *_pimpl->_duration;
  }
}

//=============================================================================
bool Reservation::is_indefinite() const
{
  return !_pimpl->_finish_time.has_value() && !_pimpl->_duration.has_value();
}

//=============================================================================
const std::string Reservation::resource_name() const
{
  return _pimpl->_resource_name;
}

//=============================================================================
ReservationId Reservation::reservation_id() const
{
  return _pimpl->_res_id;
}

//=============================================================================
schedule::ParticipantId Reservation::participant_id() const
{
  return _pimpl->_pid;
}

//=============================================================================
bool Reservation::operator==(const Reservation& other) const
{
  return other._pimpl->_res_id == _pimpl->_res_id
    && other._pimpl->_start_time == _pimpl->_start_time
    && other._pimpl->_pid == _pimpl->_pid
    && other._pimpl->_resource_name == _pimpl->_resource_name
    && other._pimpl->_duration == _pimpl->_duration
    && other._pimpl->_finish_time == _pimpl->_finish_time;
}

//=============================================================================
Reservation Reservation::make_reservation(
  rmf_traffic::Time start_time,
  std::string resource_name,
  schedule::ParticipantId pid,
  std::optional<rmf_traffic::Duration> duration,
  std::optional<rmf_traffic::Time> finish_time)
{
  static std::atomic<uint64_t> counter {0};
  Reservation res;

  res._pimpl->_res_id = counter;
  res._pimpl->_start_time = start_time;
  res._pimpl->_pid = pid;
  res._pimpl->_resource_name = resource_name;
  res._pimpl->_duration = duration;
  res._pimpl->_finish_time = finish_time;
  
  counter++;

  return res;
}

//=============================================================================
Reservation::Reservation(): 
  _pimpl(rmf_utils::make_impl<Implementation>())
{

}

}
}