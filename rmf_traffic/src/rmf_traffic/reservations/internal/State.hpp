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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE_HPP

#include <rmf_traffic/reservations/Database.hpp>
#include <unordered_map>
#include <unordered_set>
#include <map>

#include "RequestQueue.hpp"

namespace rmf_traffic {
namespace reservations {

//TODO: Currrently uses a lot of copying. In future we should use a parent based
//hierarchy
class State
{
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
      auto h1 = std::hash<T1>{}(p.first);
      auto h2 = std::hash<T2>{}(p.second);
      return h1 ^ h2;
    }
  };
public:
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  using ParticipantAssignment = std::unordered_map<
    ParticipantId, std::unordered_map<RequestId, ReservationId>>;
  using ReservationAssignment = std::unordered_map<
    ParticipantId, std::unordered_map<ReservationId, RequestId>>;

  using ReservationTimings = std::unordered_map<ReservationId, Time>;
  using ReservationResources = std::unordered_map<ReservationId, std::string>;
  using UnassignedSet = std::unordered_set<
    std::pair<ParticipantId, RequestId>, pair_hash>;

  State(std::shared_ptr<const RequestQueue> queue) :
    _queue(queue)
  {
    //Do nothing
  }
  /// Adds a request to the state
  State add_request(
    ParticipantId pid,
    RequestId reqid) const
  {
    State new_state(*this);
    new_state._unassigned.insert({pid, reqid});
    return new_state;
  }

  State remove_request(
    ParticipantId pid,
    RequestId reqid) const
  {
    State new_state(*this);
    auto req = new_state._unassigned.find({pid, reqid});

    if(req != new_state._unassigned.end())
    {
      new_state._unassigned.erase(req);
      return new_state;
    }
    auto reservation_id = new_state._assignments[pid][reqid];
    auto findings = new_state._reservation_timings.find(reservation_id);
    if(findings == new_state._reservation_timings.end())
    {
      // Non existant pid-reqid pair
      return;
    }
    auto time = findings->second;
    auto resource = new_state._reservation_resources[reservation_id];

    new_state._resource_schedules[resource].erase(time);
    new_state._reservation_resources.erase(reservation_id);
    new_state._reservation_assignment.erase(reservation_id);
    new_state._reservation_timings.erase(findings);

    return new_state;
  }

  std::optional<State> delay_reservations_after(ReservationId id, Duration dur)
  {
    State new_state(*this);
    auto resource = new_state._reservation_resources.find(id);
    if(resource == new_state._reservation_resources.end())
    {
      // This is not possible
      return std::nullopt;
    }
    auto start_time = new_state._reservation_timings[id];
    auto res_iter = new_state._resource_schedules[resource->second]
      .find(start_time);
    auto prev_iter = res_iter;
    std::next(res_iter);

    // TODO: This is extremely inefficient
    std::map<Time, ReservationId> new_times;
    auto gap_left = dur;
    while(
      res_iter != new_state._resource_schedules[resource->second].end()
      && prev_iter->second.actual_finish_time().has_value()
      && gap_left.count() > 0)
    {
      auto new_time = res_iter->first + gap_left;
      auto gap = res_iter->first - prev_iter->second.actual_finish_time().value();
      if(gap < gap_left)
        gap_left -= gap;
      else
        gap_left -= gap_left;
      new_times[new_time] = res_iter->second.reservation_id();
      prev_iter = res_iter;
      std::next(res_iter);
    }

    for(auto iter = new_times.rbegin(); iter != new_times.rend(); iter++)
    {
      bool ok  = new_state.shift_reservation_start_time(
        resource->second,
        iter->second,
        iter->first);

      if(!ok) return std::nullopt;
    }

    return {new_state}; //RIP another invocation of copy constructor
  }

  std::optional<State>
    bring_forward_reservations_before(ReservationId id, Duration dur)
  {

  }

  bool operator==(State& other) const
  {

  }

  std::size_t hash()
  {

  }

  State(const State& other) :
    _resource_schedules(other._resource_schedules),
    _assignments(other._assignments),
    _unassigned(other._unassigned),
    _reservation_timings(other._reservation_timings),
    _queue(other._queue),
    _reservation_resources(other._reservation_resources),
    _reservation_assignment(other._reservation_assignment)
  {
    // Do nothing
  }
private:
  // Returns true if a shift could be successfully applied. Else returns false.
  bool shift_reservation_start_time(
    std::string resource,
    ReservationId res_id,
    rmf_traffic::Time new_time)
  {
    auto original_time = _reservation_timings[res_id];
    auto new_start =
      _resource_schedules[resource][original_time]
        .propose_new_start_time(new_time);

    _resource_schedules[resource].erase(original_time);
    _reservation_timings[res_id] = new_time;
  }

  
  ResourceSchedules _resource_schedules;
  ParticipantAssignment _assignments;
  ReservationAssignment _reservation_assignment;
  UnassignedSet _unassigned;
  ReservationTimings _reservation_timings;
  ReservationResources _reservation_resources;
  std::shared_ptr<const RequestQueue> _queue;
};

}
}
#endif