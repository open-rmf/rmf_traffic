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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTSTORE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTSTORE_HPP

#include <rmf_traffic/reservations/ReservationRequest.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>
#include <unordered_map>

#include <iostream>

namespace rmf_traffic {
namespace reservations {

class RequestStore
{
public:
  struct RequestInfo
  {
    int priority;
    std::vector<ReservationRequest> request_options;
  };

  //============================================================================
  std::optional<RequestInfo> get_request_info(
    ParticipantId pid,
    RequestId req)
  {
    if (_reservation_info.count(pid) != 0
      && _reservation_info.at(pid).count(req) != 0)
      return _reservation_info[pid][req];
    return std::nullopt;
  }

  //============================================================================
  RequestStore()
  {
    //Do nothing
  }

  //============================================================================
  RequestStore(const rmf_traffic::reservations::RequestStore& other)
  : _reservation_info(other._reservation_info)
  {
    //Do nothing
  }

  //============================================================================
  void enqueue_reservation(
    ParticipantId pid,
    RequestId req,
    int priority,
    std::vector<ReservationRequest>& reservation
  )
  {
    _reservation_info[pid].insert({
        req,
        {
          priority,
          reservation
        }
      });
  }

  //============================================================================
  void erase_participant_requests(
    ParticipantId pid
  )
  {
    auto entry = _reservation_info.find(pid);
    if (entry == _reservation_info.end())
      return;
    _reservation_info.erase(entry);
  }

  //============================================================================
  void cancel(ParticipantId pid, RequestId req)
  {
    _reservation_info[pid].erase(req);
  }

  //============================================================================
  bool satisfies(ReservationRequest& req, Reservation reservation)
  {
    //Check upper bound
    if (req.start_time()->lower_bound().has_value())
    {
      if (reservation.start_time() < req.start_time()->lower_bound().value())
      {
        return false;
      }
    }

    //Check upper bound
    if (req.start_time()->upper_bound().has_value())
    {
      if (reservation.start_time() > req.start_time()->upper_bound().value())
      {
        return false;
      }
    }

    if (req.duration().has_value())
    {
      if (reservation.is_indefinite())
      {
        return false;
      }
      if (req.duration().value()
        > *reservation.actual_finish_time() - reservation.start_time())
      {
        return false;
      }
    }

    if (req.finish_time().has_value())
    {
      if (reservation.is_indefinite())
      {
        return false;
      }
      if (req.finish_time().value() > *reservation.actual_finish_time())
      {
        return false;
      }
    }

    if ((req.is_indefinite() && !reservation.is_indefinite())
      || (!req.is_indefinite() && reservation.is_indefinite()))
    {
      return false;
    }

    if (req.resource_name() != reservation.resource_name())
    {
      return false;
    }

    return true;
  }

  //============================================================================
  std::optional<std::size_t> satisfies(
    ParticipantId pid,
    RequestId reqid,
    Reservation& res)
  {
    for (std::size_t i = 0;
      i < _reservation_info[pid][reqid].request_options.size();
      i++)
    {
      if (satisfies(_reservation_info[pid][reqid].request_options[i], res))
        return i;
    }
    return std::nullopt;
  }

  //============================================================================
  std::size_t num_alternatives() const
  {
    std::size_t alternatives = 0;
    for (auto&[participant, requests] : _reservation_info)
    {
      for (auto&[request, request_info] : requests)
      {
        alternatives += request_info.request_options.size();
      }
    }
    return alternatives;
  }
private:
  std::unordered_map<ParticipantId,
    std::unordered_map<RequestId, RequestInfo>
  > _reservation_info;
};

} // end namespace reservations
} // end namespace rmf_traffic
#endif
