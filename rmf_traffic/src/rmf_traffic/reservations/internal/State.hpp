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

struct NextStateGenerator;

// Ripped from https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}
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

  struct ReservationAssignmentIndex
  {
    ParticipantId participant;
    RequestId reqid;
    std::size_t index;
  };
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  using ReservationAssignment = std::unordered_map<
    ParticipantId, std::unordered_map<ReservationId, RequestId>>;

  using ReservationTimings = std::unordered_map<ReservationId, Time>;
  using ReservationResources = std::unordered_map<ReservationId, std::string>;
  using ReservationRequestId = std::unordered_map<ReservationId,
    ReservationAssignmentIndex>;
  using UnassignedSet = std::unordered_set<
    std::pair<ParticipantId, RequestId>, pair_hash>;

  State(std::shared_ptr<RequestQueue> queue) :
    _queue(queue)
  {
    //Do nothing
  }

  NextStateGenerator begin()
  {
    NextStateGenerator gen;
    gen.mode = NextStateGenerator::PotentialActionMode::ASSIGN_ITEMS;
    gen.start_state = this;
    if(_unassigned.size() > 0)
    {
      auto current_request = _queue->get_request_info(
        _unassigned.begin()->first,
        _unassigned.begin()->second).request_options[0];

      gen.unassigned_iter = _unassigned.begin();
      gen.insertion_point_iter = gen.get_search_start(current_request);
      gen.insertion_point_end = gen.get_search_end(current_request);

      State next_state(*this);
      auto new_res = [=]() -> Reservation {
        if(gen.insertion_point_end != gen.insertion_point_iter)
        {
          return Reservation::make_reservation(
            gen.insertion_point_iter->second.actual_finish_time().value(),
            resource_name,
            current_request.duration(),
            current_request.finish_time()
          );
        }
        else
        {
          if(current_request.start_time().has_value())
          {
            return Reservation::make_reservation(
              current_request.start_time().value(),
              resource_name,
              current_request.duration(),
              current_request.finish_time()
            );
          }
          else
          {
            //TODO some API for time syncing
          }
        }
      }();
      next_state.assign_reservation(
        unassigned_iter->first,
        unassigned_iter->second,
        new_res
      )
      gen.next_state = next_state;
    }
    gen.remove_resource_iter = _resource_schedules.begin();
    if(_resource_schedules.size() > 0)
    {
      gen.remove_iter = gen.remove_resource_iter->first;
    }
    return gen;
  }

  NextStateGenerator end()
  {
    NextStateGenerator gen;
    gen.mode == NextStateGenerator::PotentialActionMode::END;
    return gen;
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
    auto reservation_id = new_state._reservation_assignments[pid][reqid];
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
    new_state._reservation_assignments.erase(reservation_id);
    new_state._reservation_timings.erase(findings);
    return new_state;
  }

  void purge_up_to_time(Time time)
  {

  }

  void debug_state()
  {
    for(auto &[resource, schedule]: _resource_schedules)
    {
      std::cout << "________________________" <<std::endl;
      std::cout << "Resource: " << std::endl;
      for(auto &[time, reservation]: schedule)
      {
        std::cout << "\t======"
        std::cout << "\tReservation: " << reservation.reservation_id() << std::endl;
        std::cout << "\tStart time: " << time.time_since_epoch().count()/1e9<< std::endl;
        if(!reservation.is_indefinite())
          std::cout << "\tEnd time: " <<
            reservation.actual_finish_time()->time_since_epoch().count()/1e9
            << std::endl;
        std::cout << "\tParticipant: " <<
          _reservation_request_ids[reservation.reservation_id()].participant
          << std::endl;
        std::cout << "\tRequestID: " <<
          _reservation_request_ids[reservation.resource_name()].request
          << std::endl;
      }
    }
  }

  bool operator==(State& other) const
  {
    return _resource_schedules == other._resource_schedules
      && _reservation_assignments == other._reservation_assignments;
  }

  std::size_t hash()
  {
    std::size_t seed = 0x928193843;
    for(auto &[resource, schedule]: _resource_schedules)
    {
      for(auto &[time, reservation]: schedule)
      {
        hash_combine(seed, time.time_since_epoch().count());
        hash_combine(
          seed,
          _reservation_request_ids[reservation.reservation_id()].participant);
        hash_combine(
          seed,
          _reservation_request_ids[reservation.reservation_id()].reqid);
      }
    }
    return seed;
  }

  State(const State& other) :
    _resource_schedules(other._resource_schedules),
    _unassigned(other._unassigned),
    _reservation_timings(other._reservation_timings),
    _queue(other._queue),
    _reservation_resources(other._reservation_resources),
    _reservation_assignments(other._reservation_assignments),
    _reservation_request_ids(other._reservation_request_ids)
  {
    // Do nothing
  }

private:
  void push_back(
    std::string resource,
    ResourceSchedule::const_iterator res_iter,
    Duration dur
  )
  {
    auto prev_iter = res_iter;
    std::next(res_iter);

    // TODO: This is extremely inefficient. Do in-place.
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
  }
  // Returns true if a shift could be successfully applied. Else returns false.
  void unassign_reservation(ReservationId res_id, std::string resource_name)
  {
    auto time =  _reservation_timings[res_id];
    _reservation_timings.erase(res_id);
    _reservation_resources.erase(res_id);
    auto request = _reservation_request_ids[res_id];

    _unassigned.insert({request.participant , request.reqid});
    _reservation_assignments[request.participant].erase(request.reqid);
    _reservation_request_ids.erase(res_id);
    _resource_schedules[resource_name].erase(time);
  }

  bool assign_reservation(
    ParticipantId part_id,
    RequestId req_id,
    Reservation res)
  {
    auto index = _queue->satisfies(part_id, req_id, res);
    if(!index.has_value())
    {
      return false;
    }

    if(res.is_indefinite())
    {
      //infinite reservation. Must be last
      auto next_reservation =
        _resource_schedules[res.resource_name()].lower_bound(
          res.start_time()
        );
      if(next_reservation != _resource_schedules[res.resource_name()].end())
      {
        return false;
      }
    }
    else
    {
      // Get the insertion point
      auto next_reservation = _resource_schedules[res.resource_name()].lower_bound(
        res.start_time()
      );
      if(next_reservation != _resource_schedules[res.resource_name()].end()
        && res.actual_finish_time().has_value()
        && next_reservation->second.start_time() < res.actual_finish_time().value())
      {
        push_back(
          res.resource_name(),
          next_reservation,
          res.actual_finish_time().value());
      }

    }

    _resource_schedules[res.resource_name()].insert({
      res.start_time(), res});
    _reservation_timings.insert({res.reservation_id(), res.start_time()});
    _reservation_assignments[part_id][req_id] = res.reservation_id();
    _reservation_resources[res.reservation_id()] = res.resource_name();
    _reservation_request_ids.insert({
      res.reservation_id(),
      {
        part_id,
        req_id,
        index.value()
      }});
    _unassigned.erase({part_id, req_id});
    return true;
  }

  bool shift_reservation_start_time(
    std::string resource,
    ReservationId res_id,
    rmf_traffic::Time new_time)
  {
    auto original_time = _reservation_timings[res_id];
    auto new_reservation =
      _resource_schedules[resource][original_time]
        .propose_new_start_time(new_time);
    auto request_info = _reservation_request_ids[res_id];
    auto index = _queue->satisfies(
      request_info.participant,
      request_info.reqid,
      new_reservation);
    if(!index.has_value())
    {
      return false;
    }
    _reservation_request_ids[res_id].reqid = *index;
    _resource_schedules[resource].erase(original_time);
    _resource_schedules[resource][new_time] = new_reservation;
    _reservation_timings[res_id] = new_time;
    return true;
  }

  ResourceSchedules _resource_schedules; // resource_name => Schedules
  ReservationAssignment _reservation_assignments; // {participant, req_id} => {reservation_id
  UnassignedSet _unassigned; // {participant, req_id}
  ReservationTimings _reservation_timings; // reservation => time
  ReservationResources _reservation_resources; // reservation => resource
  ReservationRequestId _reservation_request_ids; // reservation => {part_id, req_id, index}

  std::shared_ptr<RequestQueue> _queue;
  friend NextStateGenerator;
};
struct NextStateGenerator
{
  //TODO: RAW_POINTER-FOO
  State* start_state;
  std::optional<State> visiting_state;
  enum PotentialActionMode
  {
    ASSIGN_ITEMS,
    REMOVE_ITEMS,
    END
  };

  PotentialActionMode mode = ASSIGN_ITEMS;

  // For iterating through assignments
  State::UnassignedSet::const_iterator unassigned_iter;
  State::ResourceSchedule::const_iterator insertion_point_iter;
  State::ResourceSchedule::const_iterator insertion_point_end;
  std::size_t request_index_iter = 0;

  // For iterating through removals
  State::ResourceSchedules::const_iterator remove_resource_iter;
  State::ResourceSchedule::const_iterator remove_iter;
  std::size_t request_index = 0;

  using difference_type = std::ptrdiff_t;
  using value_type = State;
  using pointer = State*;
  using reference = State;
  using iterator_category= std::input_iterator_tag;

  State operator*() const {
    return *visiting_state;
  }

  State::ResourceSchedule::const_iterator
    get_search_start(const ReservationRequest& req)
  {
    if(req.start_time().has_value() &&
      req.start_time().value().lower_bound().has_value())
    {
      return start_state->
        _resource_schedules[req.resource_name()]
        .lower_bound(
          req.start_time().value().lower_bound().value());
    }
    else
    {
      return start_state->_resource_schedules[req.resource_name()].begin();
    }
  }

  State::ResourceSchedule::const_iterator
    get_search_end(const ReservationRequest& req)
  {
    if(req.start_time().has_value() &&
      req.start_time().value().lower_bound().has_value())
    {
      return start_state->
        _resource_schedules[req.resource_name()]
        .upper_bound(
          req.start_time().value().upper_bound().value());
    }
    else
    {
      return start_state->_resource_schedules[req.resource_name()].end();
    }
  }

  std::optional<State> advance_assignments()
  {
    while(true)
    {
      if(unassigned_iter == start_state->_unassigned.end())
      {
        return std::nullopt;
      }
      auto request_info = start_state->_queue->get_request_info(
        unassigned_iter->first,
        unassigned_iter->second);

      while(request_index_iter >= request_info.request_options.size())
      {
        request_index_iter = 0;
        unassigned_iter++;
        request_info = start_state->_queue->get_request_info(
          unassigned_iter->first,
          unassigned_iter->second);

        if(request_info.request_options.size() > 0)
        {
          insertion_point_iter =
            get_search_start(request_info.request_options[request_index]);
          insertion_point_end =
            get_search_end(request_info.request_options[request_index]);
        }
      }

      while(insertion_point_iter == insertion_point_end)
      {
        request_index_iter++;
        request_info = start_state->_queue->get_request_info(
          unassigned_iter->first,
          unassigned_iter->second);

        if(request_info.request_options.size() > 0)
        {
          insertion_point_iter =
            get_search_start(request_info.request_options[request_index]);
          insertion_point_end =
            get_search_end(request_info.request_options[request_index]);
        }
      }

      auto resource_name =
        request_info.request_options[request_index_iter].resource_name();
      auto current_request = request_info.request_options[request_index];

      State new_state(*start_state);

      if(insertion_point_iter->second.actual_finish_time().has_value())
      {
        Reservation new_res = Reservation::make_reservation(
          insertion_point_iter->second.actual_finish_time().value(),
          resource_name,
          current_request.duration(),
          current_request.finish_time()
        );
        bool ok = new_state.assign_reservation(
          unassigned_iter->first,
          unassigned_iter->second,
          new_res
        );
        if(ok)
        {
          insertion_point_iter++;
          return new_state;
        }
      }
      insertion_point_iter++;
    }
  }

  std::optional<State> advance_removals()
  {
    if(remove_resource_iter == start_state->_resource_schedules.end())
    {
      return std::nullopt;
    }
    while(remove_iter == remove_resource_iter->second.end())
    {
      remove_resource_iter++;
      remove_iter = remove_resource_iter->second.begin();
    }

    State new_state(*start_state);
    new_state.unassign_reservation(
      remove_iter->second.reservation_id(),
      remove_iter->second.resource_name());
    remove_iter++;
    return new_state;
  }

  std::optional<State> next_state() {
    auto assignment = advance_assignments();
    if(assignment.has_value())
      return assignment;
    mode = PotentialActionMode::REMOVE_ITEMS;
    auto removals = advance_removals();
    if(removals.has_value())
      return removals;
    mode = PotentialActionMode::END;
    return std::nullopt;
  }

  NextStateGenerator& operator++() {
    visiting_state = next_state();
    return *this;
  }

  NextStateGenerator operator++(int) {
    NextStateGenerator r = *this;
    ++(*this);
    return r;
  }

  bool operator==(const NextStateGenerator& other)
  {
    if(mode == PotentialActionMode::END
    && other.mode == PotentialActionMode::END)
    {
      return true;
    }
    return false;
  }

  bool operator!=(const NextStateGenerator& other)
  {
    return !(*this == other);
  }
};

}
}
#endif