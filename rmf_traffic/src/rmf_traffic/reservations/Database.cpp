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

#include <rmf_traffic/reservations/Database.hpp>
#include <set>
#include <map>

namespace rmf_traffic {
namespace reservations {

class Database::Implementation
{
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules;
  
  using ReservationMapping = std::unordered_map<ReservationId, std::string>;
  ReservationMapping _reservation_mapping;

  using GapList = std::map<rmf_traffic::Duration, std::pair<ReservationId, ReservationId>>;
  std::unordered_map<std::string, GapList> _timeGapList;

  class RequestStatus
  {
  public:
    std::vector<ReservationRequest> requests;
    std::shared_ptr<Negotiator> negotiator;
    int assigned_index;
  };

  using RequestId = uint64_t;
  using RequestTracker = std::unordered_map<RequestId, RequestStatus>;
  RequestTracker _request_tracking;

  using ConflictTracker = std::unordered_map<ReservationId, std::vector<RequestId>>;
  ConflictTracker _conflict_tracker;

public:

  bool can_accomodate(ReservationRequest& request)
  {
    auto name = request.resource_name();
    auto resource_schedule = _resource_schedules.find(name);

    if(resource_schedule == _resource_schedules.end())
    {
      // Resource has not yet been created, you may reserve it
      return true;
    }

    if(resource_schedule->second.size() == 0)
    {
      // Resource has been alocated but no reservations have been made yet
      return true;
    }

    if(!request.start_time().has_value())
    {
      // Handles the case where no start time is specified;

      // Check if there is an indefinite reservation.
      auto last_reservation = resource_schedule->second.rbegin();
      if(last_reservation->second.is_indefinite())
      {
        if(!request.duration().has_value())
        {
          if(!request.finish_time().has_value())
          {
            // Requesting an indefinite reservation - not possible
            // as theres already one
            return false;
          }

          auto slot_after = resource_schedule->second.upper_bound(*request.finish_time());          
          if(slot_after == resource_schedule->second.end())
          {
            // The reservation is after the indefinite reservation
            return false;
          }

        }
        // Since, last reservation was indefinite reservation, We can't 
        // insert any thing afterwards, we will have to check if there are time gaps
        // between now and the reservation.
        if(resource_schedule->second.size() == 1)
        {
          // Theres only one reservation for this resource. Hence, the gaplist will
          // not have been populated. We will simply check if the request ends
          // before the start of the indefinite reservation.
          return request.finish_time() <= last_reservation->second.start_time();
        }
        auto gap = _timeGapList[name].upper_bound(*request.duration());
      }
      else
      {
        // We may simply place the reservation after the last reservation
        return true;
      }
    }

    if(!request.finish_time().has_value() && !request.duration().has_value())
    {
      // Handle indefinite reservations
      auto last_reservation = resource_schedule->second.rbegin();
      if(last_reservation->second.is_indefinite())
      {
        // Last reservation was indefinite reservation. Can't 
        // insert second indefinite reservation
        return false;
      }

      // Check end time of last reservation
      auto end_time = *last_reservation->second.actual_finish_time();
  
      return end_time <= request.start_time()->upper_bound();
    }


  }
};

void Database::make_reservation(
  std::vector<ReservationRequest> request,
  std::shared_ptr<Negotiator> nego)
{
  
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

}
}