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
#ifndef SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_CONSTRAIN_TRACKER
#define SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_CONSTRAIN_TRACKER
#include <rmf_traffic/reservations/Database.hpp>
#include <set>
#include <map>
#include <future>
#include <execution>

#include <iostream>

namespace rmf_traffic {
namespace reservations {

class ConstraintTracker
{
public:

  class RequestStatus
  {
  public:
    enum Status
    {
      Pending,
      Assigned
    };
    std::vector<ReservationRequest> requests;
    std::shared_ptr<Negotiator> negotiator;
    Status status;
    int assigned_index;
    int priority;
    ReservationId assigned_reservation;
  };

  using RequestId = uint64_t;
  using RequestTracker = std::unordered_map<RequestId, RequestStatus>;
  RequestTracker _request_tracker;

  using ConflictTracker = std::unordered_map<ReservationId, std::unordered_set<RequestId>>;
  ConflictTracker _conflict_tracker;

  using ActiveReservationTracker = std::unordered_map<ReservationId, RequestId>;
  ActiveReservationTracker _active_reservation_tracker;

  bool satisfies(ReservationRequest& req, Reservation reservation)
  {
    //Check upper bound
    if(req.start_time()->lower_bound().has_value())
    {
      if(reservation.start_time() < req.start_time()->lower_bound().value())
      {
        return false;
      }
    }

    //Check upper bound
    if(req.start_time()->upper_bound().has_value())
    {
      if(reservation.start_time() > req.start_time()->upper_bound().value())
      {
        return false;
      }
    }

    if(req.duration().has_value())
    {
      if(reservation.is_indefinite())
      {
        return false;
      }
      if(req.duration().value()
        > *reservation.actual_finish_time() - reservation.start_time())
      {
        return false;
      }
    }

    if(req.finish_time().has_value())
    {
      if(reservation.is_indefinite())
      {
        return false;
      }
      if(req.finish_time().value() > *reservation.actual_finish_time())
      {
        return false;
      }
    }

    if(req.is_indefinite() && !reservation.is_indefinite()
      || !req.is_indefinite() && reservation.is_indefinite())
    {
      return false;
    }

    if(req.resource_name() != reservation.resource_name())
    {
      return false;
    }

    return true;
  }

  std::optional<std::size_t> satisfies(
    RequestId req_id,
    Reservation reservation)
  {
    std::size_t i = 0;
    while(
      i <  _request_tracker[req_id].requests.size() &&
      !satisfies(_request_tracker[req_id].requests[i], reservation))
    {
      i++;
    }
    if(i >= _request_tracker[req_id].requests.size())
    {
      // None of the constraints are satisfied by current reservation
      return std::nullopt;
    }
    return {i};
  }

  RequestId add_request_queue(
    std::vector<ReservationRequest>& requests,
    std::shared_ptr<Negotiator> negotiator)
  {
    static std::atomic<RequestId> counter{0};
    _request_tracker.insert({counter, RequestStatus {
      std::move(requests),
      negotiator,
      RequestStatus::Status::Pending,
      0
    }});
    return counter++;
  }

  void associate_request_with_reservation(
    RequestId req_id,
    Reservation& reservation)
  {
    auto i = satisfies(req_id, reservation);
    if(!i.has_value()) return;
    _request_tracker[req_id].assigned_index = *i;
    _request_tracker[req_id].status = RequestStatus::Status::Assigned;
    _request_tracker[req_id].assigned_reservation =
      reservation.reservation_id();
    auto reservation_id = reservation.reservation_id();
    auto resource =  reservation.resource_name();
    _active_reservation_tracker[reservation_id] = req_id;
  }

  bool remove_request(RequestId req)
  {
    auto it = _request_tracker.find(req);
    if(it == _request_tracker.end())
    {
      return false;
    }

    if(it->second.status == RequestStatus::Status::Assigned)
    {
      auto res_id = it->second.assigned_reservation;
      _active_reservation_tracker.erase(res_id);
    }

    _request_tracker.erase(it);
    return true;
  }

  std::optional<ReservationId> get_associated_reservation(RequestId req)
  {
    auto it = _request_tracker.find(req);
    if(it == _request_tracker.end())
    {
      return std::nullopt;
    }

    if(it->second.status != RequestStatus::Status::Assigned)
    {
      return std::nullopt;
    }

    return it->second.assigned_reservation;
  }

  std::optional<RequestId> get_associated_constraint(ReservationId id)
  {
    auto it = _active_reservation_tracker.find(id);
    if(it == _active_reservation_tracker.end())
      return std::nullopt;
    return it->second;
  }
};
}
}

#endif
