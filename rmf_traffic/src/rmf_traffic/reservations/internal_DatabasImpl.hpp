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
#ifndef SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_DATABASE
#define SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_DATABASE
#include <rmf_traffic/reservations/Database.hpp>
#include <set>
#include <map>
#include <future>
#include <execution>

#include <iostream>

namespace rmf_traffic {
namespace reservations {

class Database::Implementation
{
public:
  //===========================================================================
  // Storage Classes - These classes store the actual reservation
  // and the requests
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules;

  using ReservationMapping = std::unordered_map<ReservationId, std::string>;
  ReservationMapping _reservation_mapping;

  class RequestStatus
  {
  public:
    enum Status
    {
      Pending,
      assigned
    };
    std::vector<ReservationRequest> requests;
    std::shared_ptr<Negotiator> negotiator;
    Status status;
    int assigned_index;
  };

  using RequestId = uint64_t;
  using RequestTracker = std::unordered_map<RequestId, RequestStatus>;
  RequestTracker _request_tracker;

  using ConflictTracker = std::unordered_map<ReservationId, std::unordered_set<RequestId>>;
  ConflictTracker _conflict_tracker;

  using ActiveReservationTracker = std::unordered_map<ReservationId, RequestId>;
  ActiveReservationTracker _active_reservation_tracker;

  ///==========================================================================
  /// Given an active reservation ID lookup the request id.
  const ReservationRequest lookup_request(ReservationId res_id)
  {
    auto req_id = _active_reservation_tracker[res_id];
    return _request_tracker[req_id].requests[_request_tracker[req_id].assigned_index];
  }

  ///==========================================================================
  /// Given an active reservation ID lookup the negotiator
  const std::shared_ptr<Negotiator> lookup_negotiator(ReservationId res_id)
  {
    auto req_id = _active_reservation_tracker[res_id];
    return _request_tracker[req_id].negotiator;
  }

  bool contains_indefinite_resolution(ResourceSchedule& sched)
  {
    auto res = sched.rbegin();
    if(res == sched.rend()) {
      return false;
    }
    return res->second.is_indefinite();
  }

  //Check if a reservation satisfies a request. This is useful later on
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
  

  class Plan
  {
  public:
    std::vector<Reservation> push_back_reservations;
    std::vector<Reservation> bring_forward_reservations;
    int num_conflict;
    float cost;
  };
  /// \brief satisfies that are impacted when we perform a push back
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

    assert(desired_time >  start_time);

    Plan proposal;
    auto item = sched.lower_bound(start_time);
    auto next_time = desired_time;
    while(item != sched.end() && item->first <= next_time)
    {
      auto proposed_push_back = item->second.propose_new_start_time(next_time);
      auto request_id = _active_reservation_tracker[item->second.reservation_id()];

      if(!satisfies(request_id, proposed_push_back))
      {
        return std::nullopt;
      }
      auto last_fin_time = proposed_push_back.actual_finish_time();
      proposal.push_back_reservations.push_back(proposed_push_back);
      if(!last_fin_time.has_value())
      {
        break;
      }
      next_time = last_fin_time.value();
      item = std::next(item);
    }
    return {proposal};
  }

  /// \brief Gives a list of reservations that are impacted when we perform a push back
  /// \param sched is the ResourceSchedule to operate on
  /// \param start_time is the time after which all the reservations will be pushed back
  /// \param desired_time is the time which the reservation should be moved to .
  /// \returns a Plan if a valid plan exists. IF a valid bring forward exists.
  std::optional<Plan> 
    plan_bring_forward_reservations(ResourceSchedule& sched,
      rmf_traffic::Time start_time,
      rmf_traffic::Time desired_time,
      RequestId request_id)
  {

    assert(desired_time < start_time);

    Plan proposal;
    auto item = sched.lower_bound(start_time);
    auto next_time = desired_time;
    while(item != sched.end() && item->first > next_time)
    {
      auto proposed_push_back = item->second.propose_new_start_time(next_time);
      auto request_id = _active_reservation_tracker[item->second.reservation_id()];

      if(!satisfies(request_id, proposed_push_back))
      {
        return std::nullopt;
      }
      auto last_fin_time = proposed_push_back.actual_finish_time();
      proposal.push_back_reservations.push_back(proposed_push_back);
      if(!last_fin_time.has_value())
      {
        break;
      }
      next_time = last_fin_time.value();
      item = std::prev(item);
    }
    return {proposal};
  }

  /// Determines the earliest starting time given a reservation
  /// TODO: We can come up with some cacheing scheme to make things faster
  ///   Creating hash maps is a huge waste of time.
  const std::unordered_map<ReservationId, std::optional<Time>> 
    earliest_time_computation(ResourceSchedule& schedule)
  {
    std::unordered_map<ReservationId, std::optional<Time>> result;

    for(auto it = schedule.begin(); it != schedule.end(); it++)
    {
      const auto& [time, reservation] = *it;
      if(result.size() == 0)
      {
        auto request = lookup_request(reservation.reservation_id());
        if(request.start_time().has_value())
        {
          result[reservation.reservation_id()] = request.start_time()->lower_bound();
        }
        else
        {
          result[reservation.reservation_id()] = std::nullopt;
        }
      }
      else
      {
        auto prev_it = std::prev(it);
        const auto& prev_reservation = prev_it->second;
        auto prev_time = prev_it->first;

        // Get the earliest end time of the previous reservation.
        auto prev_earliest_end = [=]() -> std::optional<Time> 
        {
          if(!result.find(prev_reservation.reservation_id())->second.has_value())
          {
            return std::nullopt;
          }

          auto request_params = lookup_request(prev_reservation.reservation_id());

          if(!request_params.duration().has_value())
          {
            return request_params.finish_time();
          }

          return {*request_params.duration() + prev_time};
        }();

        auto request_params 
          = lookup_request(reservation.reservation_id());

        if(!request_params.start_time().has_value()
          || !request_params.start_time()->lower_bound().has_value())
        {
          // This reservation has no lower bound.
          // Other reservation must be earlier
          result[reservation.reservation_id()] = prev_earliest_end;
        }
        else
        {
          if(prev_earliest_end.has_value())
          {
            result[reservation.reservation_id()]
              = request_params.start_time()->lower_bound();
          }
          else
          {
            result[reservation.reservation_id()] 
              = std::max(*prev_earliest_end, *request_params.start_time()->lower_bound());
          }
        }
      }
    }

    return result;
  }

  const std::unordered_map<ReservationId, std::optional<Time>>
    latest_start_time_computation(ResourceSchedule& schedule)
  {
    std::unordered_map<ReservationId, std::optional<Time>> result;

    for(auto it = schedule.rbegin(); it != schedule.rend(); it++)
    {
      const auto& [time, reservation] = *it;
      if(result.size() == 0)
      {
        auto request = lookup_request(reservation.reservation_id());
        if(request.start_time().has_value())
        {
          result[reservation.reservation_id()] = request.start_time()->upper_bound();
        }
        else
        {
          result[reservation.reservation_id()] = std::nullopt;
        }
      }
      else
      {
        // Reverse iterator prev actually means next in this case
        auto prev_it = std::prev(it);
        const auto& prev_reservation = prev_it->second;
        auto prev_time = prev_it->first;
        
        // Get the latest start time of the next reservation.
        auto get_latest_start = [=](const Reservation& res) -> std::optional<Time> 
        {
          auto request_params = lookup_request(res.reservation_id());
          if(!request_params.start_time().has_value())
            return std::nullopt;

          return request_params.start_time()->upper_bound();
        };

        auto prev_latest_start = get_latest_start(prev_reservation);
        auto curr_latest_start = get_latest_start(reservation);

        if(!prev_latest_start.has_value())
        {
          result[reservation.reservation_id()] = curr_latest_start;
        }
        else 
        {
          if(!curr_latest_start.has_value())
          {
            result[reservation.reservation_id()] = prev_latest_start;
          }
          else
          {
            result[reservation.reservation_id()] 
              = {std::min(*prev_latest_start, *curr_latest_start)};
          }
        }
        
      }
    }

    return result;
  }

  bool satisfies_request_start_constraints(Time time, ReservationRequest req)
  {
    auto start = req.start_time();
    if(!start.has_value())
    {
      return true;
    }

    if(start->lower_bound().has_value() &&
      time < start->lower_bound())
    {
      return false;
    }

    if(start->upper_bound().has_value() &&
      time > start->upper_bound())
    {
      return false;
    }

    return true;
  }

  std::vector<std::optional<Duration>> nth_conflict_times_bring_forward(
    ResourceSchedule& sched,
    ResourceSchedule::const_iterator iter,
    Time desired_time,
    Time time_limit
  )
  {

    int conflicts = 0;
    std::vector<std::optional<Duration>> conflict_table;
    Duration conflict_duration = desired_time - *iter->second.actual_finish_time();
    if(conflict_duration.count() != 0)
    {
      Duration dur{0};
      conflict_table.push_back(dur);
    }
    else
    {
      conflict_table.push_back(std::nullopt);
    }

    while(iter->first >= time_limit)
    {
      if(iter==sched.begin())
      {
        break;
      }
      auto _prev_res = std::prev(iter);
      auto gap = iter->first - *_prev_res->second.actual_finish_time();
      conflict_duration += gap;
      if(gap.count() == 0)
      {
        // There is no gap so it is not possible to have n conflicts
        conflict_table[conflict_table.size() - 1] = std::nullopt;
      }
      conflict_table.push_back({conflict_duration});
      iter = _prev_res;
    }
    return conflict_table;
  }

  std::vector<std::optional<Duration>> nth_conflict_times_push_back(
    ResourceSchedule& sched,
    ResourceSchedule::const_iterator iter,
    Time desired_time,
    Time time_limit
  )
  {
    int conflicts = 0;
    std::vector<std::optional<Duration>> conflict_table;

    if(iter == sched.end()) 
    {
      Duration dur{0};
      conflict_table.push_back(dur); 
    }
    Duration conflict_duration = iter->first - desired_time;
    if(conflict_duration.count() != 0)
    {
      Duration dur{0};
      conflict_table.push_back(dur);
    }
    else
    {
      conflict_table.push_back(std::nullopt);
    }
    conflict_table.push_back({conflict_duration}); 
    while(iter->second.actual_finish_time() < time_limit)
    {
      auto _next_res = std::next(iter);
      if(_next_res==sched.end())
      { 
        break;
      }
      auto gap = *iter->second.actual_finish_time() - _next_res->first;
      //TODO: consider other constraints
      conflict_duration += gap;
      if(gap.count() == 0)
      {
        // There is no gap so it is not possible to have n conflicts
        conflict_table[conflict_table.size() - 1] = std::nullopt;
      }
      conflict_table.push_back({conflict_duration});
      iter = _next_res;
    }
    return conflict_table;
  }

  /// \returns a list of plans in ascending number of conflicts.
  /// The inner workings of this algorithm are as follows:
  /// - get the 
  /// runtime complexity O(n^2) in number of items currently on the 
  std::vector<Plan>  least_conflicts(
    Duration duration,
    ResourceSchedule& sched,
    ResourceSchedule::const_iterator iter,
    Time earliest_start_time,
    Time latest_start_time,
    RequestId req_id)
  {
    
  }

  /// \returns all reservations which conflict with the start range. 
  /// the returned plans are sorted by number of conflicts followed by
  std::vector<Plan> create_plans(
    ReservationRequest& req,
    RequestId req_id)
  {
    
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
    _request_tracker[req_id].status = RequestStatus::Status::assigned;
    auto reservation_id = reservation.reservation_id();
    auto resource =  reservation.resource_name();
    _resource_schedules[resource].insert({reservation.start_time(), reservation});
    _reservation_mapping[reservation_id] = resource;
    _active_reservation_tracker[reservation_id] = req_id;
  }


  bool approve_plan(Plan& plan)
  {
    std::mutex _mtx;
    bool ok = true;
    std::for_each(
      std::execution::par_unseq,
      plan.push_back_reservations.begin(),
      plan.push_back_reservations.end(),
      [&](Reservation& res)
      {
        auto nego = lookup_negotiator(res.participant_id());
        auto accepted = nego->offer_received(res);
        std::lock_guard lock(_mtx);
        ok &= accepted;
      }
    );
    return ok;
  }

  void commit_plan(Plan& plan)
  {

  }



  void make_reservation(
    std::vector<ReservationRequest>& request,
    std::shared_ptr<Negotiator> nego)
  {
    auto req_id = add_request_queue(request, nego);
    //create_plans()
  }
  
  void debug_reservations(ResourceSchedule& sched)
  {
    std::cout << "=======================" <<std::endl;
    for(auto x: sched)
    {
      std::cout << "--" << std::endl;
      std::cout << "Start time: " << x.first.time_since_epoch().count() << std::endl;
      std::cout << "Reservation ID: " << x.second.reservation_id() <<  std::endl;
      
      if(x.second.actual_finish_time().has_value())
        std::cout << "End time: " << x.second.actual_finish_time()->time_since_epoch().count() <<std::endl;
      else
        std::cout << "End time: " << "Nivar" <<std::endl;

      if(x.second.duration().has_value())
        std::cout << "Min duration: " << x.second.duration()->count() << std::endl;

      if(x.second.finish_time().has_value())
        std::cout << "Finish time (requested): " << x.second.finish_time()->time_since_epoch().count() << std::endl;
    }
  }

  void debug_reservations(Plan& sched)
  {

    std::cout << "=======================" <<std::endl;
    for(auto x: sched.bring_forward_reservations)
    {
      
      std::cout << "Start time: " << x.start_time().time_since_epoch().count() << std::endl;
      std::cout << "Reservation ID: " << x.reservation_id() <<  std::endl;
      
      if(x.actual_finish_time().has_value())
        std::cout << "End time: " << x.actual_finish_time()->time_since_epoch().count() <<std::endl;
      else
        std::cout << "End time: " << "Nivar" <<std::endl;

      if(x.duration().has_value())
        std::cout << "Min duration: " << x.duration()->count() << std::endl;

      if(x.finish_time().has_value())
        std::cout << "Finish time (requested): " << x.finish_time()->time_since_epoch().count() << std::endl;
    }
    for(auto x: sched.push_back_reservations)
    {
      std::cout << "--" << std::endl;
      std::cout << "Start time: " << x.start_time().time_since_epoch().count() << std::endl;
      std::cout << "Reservation ID: " << x.reservation_id() <<  std::endl;
      
      if(x.actual_finish_time().has_value())
        std::cout << "End time: " << x.actual_finish_time()->time_since_epoch().count() <<std::endl;
      else
        std::cout << "End time: " << "Nivar" <<std::endl;

      if(x.duration().has_value())
        std::cout << "Min duration: " << x.duration()->count() << std::endl;

      if(x.finish_time().has_value())
        std::cout << "Finish time (requested): " << x.finish_time()->time_since_epoch().count() << std::endl;
    }
  }

};

}
}
#endif
