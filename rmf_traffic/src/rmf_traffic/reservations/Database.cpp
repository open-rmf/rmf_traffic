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
#include <future>

namespace rmf_traffic {
namespace reservations {

class Database::Implementation
{
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules;
  
  using ReservationMapping = std::unordered_map<ReservationId, std::string>;
  ReservationMapping _reservation_mapping;

  class RequestStatus
  {
  public:
    std::vector<ReservationRequest> requests;
    std::shared_ptr<Negotiator> negotiator;
    int assigned_index;
  };

  using RequestId = uint64_t;
  using RequestTracker = std::unordered_map<RequestId, RequestStatus>;
  RequestTracker _request_tracker;

  using ConflictTracker = std::unordered_map<ReservationId, std::unordered_set<RequestId>>;
  ConflictTracker _conflict_tracker;

  using ActiveReservationTracker = std::unordered_map<ReservationId, RequestId>;
  ActiveReservationTracker _active_reservation_tracker;

  bool contains_indefinite_resolution(ResourceSchedule& sched)
  {
    auto res = sched.rbegin();
    if(res == sched.rend()) {
      return false;
    }
    return res->second.is_indefinite();
  }

  std::optional<Time> latest_start_time(ReservationId id)
  {
    RequestId req_id = _active_reservation_tracker[id];
    auto status = _request_tracker[req_id];
    auto& start_constraints = status.requests[status.assigned_index];
    
    if(!start_constraints.start_time().has_value() ||
      !start_constraints.start_time()->upper_bound().has_value()) 
      return std::nullopt;

    start_constraints.start_time()->upper_bound();
  }

public:

  rmf_traffic::Time current_time;

  class Plan
  {
  public:
    unsigned long long cost;
    std::vector<Reservation> impacted_reservations;
  };

  /// \brief Gives a list of reservations that are impacted when we perform a push back
  /// \param sched is the ResourceSchedule to operate on
  /// \param start_time is the time after which all the reservations will be pushed back
  /// \param desired_time is the time which the reservation should be moved to 
  /// \returns a Plan if a valid plan exists. IF a pushback_results in  
  std::optional<Plan> 
    plan_push_back_reservations(ResourceSchedule& sched, 
      rmf_traffic::Time start_time, 
      rmf_traffic::Time desired_time,
      RequestId request_id)
  {
    auto item = sched.lower_bound(start_time);
    auto next_item = std::next(item);
    auto item_desired_time = desired_time;

    auto push_back_duration = desired_time - item->first;

    Plan proposal;
    proposal.cost = 0;

    while(
      next_item != sched.end() && // We have not reached the end of the schedule
      next_item->first < item_desired_time+*item->second.duration() 
      // There is an overlap between next item and current item
      // We don't need to check for infinite reservations because only 
      // the last item will be infinite and this while loop doesn't handle the last item
    )
    {
      auto latest_start = latest_start_time(item->second.reservation_id());
      if(latest_start.has_value() && 
        *latest_start < item_desired_time)
      {
        // Violates the starting conditions mark this as a potential conflict
        _conflict_tracker[item->second.reservation_id()].insert(request_id);
        return std::nullopt;
      }

      auto proposed_reservation = item->second.propose_new_start_time(item_desired_time);
      proposal.impacted_reservations.push_back(proposed_reservation);
      proposal.cost += (item_desired_time - *latest_start).count();

      // Get the next_item
      item = next_item;
      item_desired_time = item->first + push_back_duration;
      next_item = std::next(next_item);
    }

    auto latest_start = latest_start_time(item->second.reservation_id());
    if(latest_start.has_value() && 
      *latest_start < item_desired_time)
    {
      // Violates the starting conditions mark this as a potential conflict
      _conflict_tracker[item->second.reservation_id()].insert(request_id);
      return std::nullopt;
    }
    auto proposed_reservation = item->second.propose_new_start_time(item_desired_time);
    proposal.impacted_reservations.push_back(proposed_reservation);
    proposal.cost += (item_desired_time - *latest_start).count();

    return proposal;
  }


  void add_reservation(Reservation& res)
  {

  }

  
  
  std::future<std::optional<Reservation>> reserve(
    ReservationRequest& request,
    std::shared_ptr<Negotiator> nego)
  {
    
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