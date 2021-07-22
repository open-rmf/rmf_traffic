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
#include <iostream>
#include <map>
#include <set>

#include "RequestStore.hpp"

namespace rmf_traffic {
namespace reservations {

struct NextStateGenerator;

class StateDiff;

// Ripped from https://stackoverflow.com/questions/2590677/how-do-i-combine-hash-values-in-c0x
template<class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
  std::hash<T> hasher;
  seed ^= hasher(v) + 0x9e3779b9 + (seed<<6) + (seed>>2);
}
///=============================================================================
/// \brief This class represents the current schedule. It is used by the planner
/// to explore the space of possible schedules. Iterating over the State allows
/// one to find out the next possible schedule based on the current schedule
/// stored in the state.
class State
{
  struct pair_hash
  {
    template<class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2>& p) const
    {
      auto h1 = std::hash<T1>{} (p.first);
      auto h2 = std::hash<T2>{} (p.second);
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
    ParticipantId, std::unordered_map<RequestId, ReservationId>>;

  using ReservationTimings = std::unordered_map<ReservationId, Time>;
  using ReservationResources = std::unordered_map<ReservationId, std::string>;
  using ReservationRequestId = std::unordered_map<ReservationId,
      ReservationAssignmentIndex>;
  using UnassignedSet = std::unordered_set<
    std::pair<ParticipantId, RequestId>, pair_hash>;

  //============================================================================
  State(std::shared_ptr<RequestStore> queue)
  : _queue(queue)
  {
    //Do nothing
  }

  //============================================================================
  NextStateGenerator begin();

  //============================================================================
  NextStateGenerator end();

  //============================================================================
  /// Adds a request to the state
  /// Note: This modifies the ReservationStore.
  State add_request(
    ParticipantId pid,
    RequestId reqid,
    int priority,
    std::vector<ReservationRequest>& reservation) const
  {
    State new_state(*this);
    new_state._unassigned.insert({pid, reqid});
    new_state._queue->enqueue_reservation(pid, reqid, priority, reservation);
    return new_state;
  }

  //============================================================================
  /// Removes a request from the current state and returns the new state.
  /// Note: This modifies the ReservationStore.
  State remove_request(
    ParticipantId pid,
    RequestId reqid) const
  {
    State new_state(*this);
    auto req = new_state._unassigned.find({pid, reqid});

    if (req != new_state._unassigned.end())
    {
      new_state._unassigned.erase(req);
      return new_state;
    }
    auto reservation_id = new_state._reservation_assignments[pid][reqid];
    auto findings = new_state._reservation_timings.find(reservation_id);
    if (findings == new_state._reservation_timings.end())
    {
      // Non existant pid-reqid pair
      return new_state;
    }
    auto time = findings->second;
    auto resource = new_state._reservation_resources[reservation_id];

    new_state._resource_schedules[resource].erase(time);
    new_state._reservation_resources.erase(reservation_id);
    new_state._reservation_assignments[pid].erase(reqid);
    new_state._reservation_timings.erase(findings);
    new_state._queue->cancel(pid, reqid);
    return new_state;
  }

  //============================================================================
  /// Remove a participant
  State remove_participant(
    ParticipantId pid)
  {
    //TODO(arjo): Lots of unessecary copying
    State new_state(*this);
    for (auto [request, reservation]: _reservation_assignments[pid])
    {
      new_state = new_state.remove_request(pid, request);
    }

    for (auto [participant_id, reqid] : _unassigned)
    {
      if(participant_id == pid)
        new_state = new_state.remove_request(participant_id, reqid);
    }
    return new_state;
  }

  //============================================================================
  void debug_state()
  {
    std::cout << "________________________" <<std::endl;
    for (auto&[resource, schedule]: _resource_schedules)
    {
      std::cout << "Resource: " << resource << std::endl;
      for (auto&[time, reservation]: schedule)
      {
        std::cout << "\t======" << std::endl;
        std::cout << "\tReservation: "
                  << reservation.reservation_id() << std::endl;
        std::cout << "\tStart time: " <<
          std::chrono::duration_cast<std::chrono::seconds>(
            time.time_since_epoch()
          ).count() << std::endl;
        if (!reservation.is_indefinite())
          std::cout << "\tEnd time: " <<
            std::chrono::duration_cast<std::chrono::seconds>(
              reservation.actual_finish_time()->time_since_epoch()
            ).count()
                    << std::endl;
        std::cout << "\tParticipant: " <<
          _reservation_request_ids[reservation.reservation_id()].participant
                  << std::endl;
        std::cout << "\tRequestID: " <<
          _reservation_request_ids[reservation.reservation_id()].reqid
                  << std::endl;
      }
    }
  }

  //============================================================================
  bool operator==(const State& other) const
  {
    return _resource_schedules == other._resource_schedules
      && _unassigned == other._unassigned;
  }

  //============================================================================
  std::size_t hash() const
  {
    std::size_t seed = 0x928193843;
    std::set<std::string> resources;

    // Make sure the order of iteration is correct
    for (auto&[resource, schedule]: _resource_schedules)
    {
      resources.insert(resource);
    }

    for (auto resource: resources)
    {
      auto schedule = _resource_schedules.find(resource)->second;
      for (auto&[time, reservation]: schedule)
      {
        hash_combine(seed, time.time_since_epoch().count());
        auto val =
          _reservation_request_ids.find(reservation.reservation_id())->second;
        hash_combine(seed, val.participant);
        hash_combine(seed, val.reqid);
      }
    }
    return seed;
  }

  //============================================================================
  void set_current_time(Time time)
  {
    _current_time = time;
  }

  //============================================================================
  UnassignedSet unassigned() const
  {
    return _unassigned;
  }

  //============================================================================
  ResourceSchedules resource_schedule() const
  {
    return _resource_schedules;
  }

  //============================================================================
  ReservationRequestId request_ids() const
  {
    return _reservation_request_ids;
  }

  //============================================================================
  bool check_if_conflicts(Reservation res) const
  {
    auto resource = res.resource_name();
    auto sched = _resource_schedules.find(resource);
    if (sched == _resource_schedules.end())
      return false;

    auto start_slot = sched->second.upper_bound(res.start_time());

    if (res.is_indefinite())
    {
      if (sched->second.size() == 0)
        return start_slot  != sched->second.end();
      else if (
        sched->second.rbegin()->second.actual_finish_time().has_value()
        && sched->second.rbegin()->second.actual_finish_time() <
        res.start_time())
        return start_slot  != sched->second.end();
      else
        return true;
    }

    auto end_slot =
      sched->second.upper_bound(
      res.actual_finish_time().value());

    if (start_slot != sched->second.begin())
    {
      auto prev_iter = std::prev(start_slot);
      // Reservation definitely has end time
      return !(end_slot == start_slot &&
        start_slot->first >= res.actual_finish_time().value() &&
        prev_iter->second.actual_finish_time().has_value() &&
        prev_iter->second.actual_finish_time().value() <= res.start_time());
    }
    else
    {
      return !(end_slot == start_slot &&
        start_slot->first >= res.actual_finish_time().value());
    }
  }


  //============================================================================
  State unassign_reservation(ParticipantId pid, RequestId reqid) const
  {
    State new_state(*this);
    auto res_id =
      _reservation_assignments.find(pid)->second.find(reqid)->second;
    auto resource = _reservation_resources.find(res_id)->second;
    new_state.unassign_reservation(res_id, resource);
    return new_state;
  }

  //============================================================================
  bool assign_reservation(
    ParticipantId part_id,
    RequestId req_id,
    Reservation res)
  {
    auto index = _queue->satisfies(part_id, req_id, res);
    if (!index.has_value())
    {
      return false;
    }

    if (res.is_indefinite())
    {
      //infinite reservation. Must be last
      auto next_reservation =
        _resource_schedules[res.resource_name()].lower_bound(
        res.start_time()
        );
      if (next_reservation != _resource_schedules[res.resource_name()].end())
      {
        return false;
      }
    }
    else
    {
      // Get the insertion point
      auto next_reservation =
        _resource_schedules[res.resource_name()].lower_bound(
        res.start_time()
        );

      if (next_reservation != _resource_schedules[res.resource_name()].end()
        && res.actual_finish_time().has_value()
        && next_reservation->second.start_time() <
        res.actual_finish_time().value())
      {
        auto ok = push_back(
          res.resource_name(),
          next_reservation,
          next_reservation->second.start_time() -
          res.actual_finish_time().value());
        if (!ok)
        {
          return false;
        }
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

  std::shared_ptr<RequestStore> requests() const
  {
    return _queue;
  }
private:
  //============================================================================
  /// \brief Unassign a reservation from the current state. Mutates the state.
  void unassign_reservation(ReservationId res_id, std::string resource_name)
  {
    auto time = _reservation_timings[res_id];
    _reservation_timings.erase(res_id);
    _reservation_resources.erase(res_id);
    auto request = _reservation_request_ids[res_id];

    _unassigned.insert({request.participant, request.reqid});
    _reservation_assignments[request.participant].erase(request.reqid);
    _reservation_request_ids.erase(res_id);
    _resource_schedules[resource_name].erase(time);
  }

  //============================================================================
  /// \brief Performs a pushback operation on the current schedule. This is used
  /// to make space for new reservations.
  /// \param resource - the resource for which we are trying to make more space
  /// \param res_iter = the point at which to perform the pushback
  /// \param dur - the duration of the pushback
  /// \returns true if the pushback is possible, false otherwise. Reasons for
  /// returning false include the fact that some constraints are not satisfied,
  /// or the duration is negative.
  bool push_back(
    std::string resource,
    ResourceSchedule::const_iterator res_iter,
    Duration dur
  )
  {
    if (dur.count() < 0) return false;

    auto prev_iter = res_iter;
    std::next(res_iter);
    // TODO: This is extremely inefficient. Do in-place.
    std::map<Time, ReservationId> new_times;
    auto gap_left = dur;
    while (
      res_iter != _resource_schedules[resource].end()
      && prev_iter->second.actual_finish_time().has_value()
      && gap_left.count() > 0)
    {
      auto new_time = res_iter->first + gap_left;
      auto gap = res_iter->first -
        prev_iter->second.actual_finish_time().value();
      if (gap < gap_left)
        gap_left -= gap;
      else
        gap_left -= gap_left;
      new_times[new_time] = res_iter->second.reservation_id();
      prev_iter = res_iter;
      std::next(res_iter);
    }

    for (auto iter = new_times.rbegin(); iter != new_times.rend(); iter++)
    {
      bool ok = shift_reservation_start_time(
        resource,
        iter->second,
        iter->first);
      if (!ok) return false;
    }

    return true;
  }

  //============================================================================
  /// \brief Move the start time of a reservation.
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
    if (!index.has_value())
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

  std::shared_ptr<RequestStore> _queue;

  Time _current_time;
  friend NextStateGenerator;
  friend StateDiff;
};
//==============================================================================
/// \brief This iterator is used to iterate over the next possible states of the
/// schedule. It does this by first attempting to assign one item from the
/// unassigned list. After that it gives states by individually removing items
/// from the schedule. This is used by the planner to get the next state.
struct NextStateGenerator
{
  //TODO: RAW_POINTER-FOO
  State* start_state;
  std::optional<State> visiting_state;

  //Potential Actions
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
  bool proceed_next_resource = false;

  // For iterating through removals
  State::ResourceSchedules::const_iterator remove_resource_iter;
  State::ResourceSchedule::const_iterator remove_iter;
  std::size_t request_index = 0;

  using difference_type = std::ptrdiff_t;
  using value_type = State;
  using pointer = State*;
  using reference = State;
  using iterator_category = std::input_iterator_tag;

  //============================================================================
  State operator*() const
  {
    return *visiting_state;
  }

  //============================================================================
  State::ResourceSchedule::const_iterator
  get_search_start(const ReservationRequest& req)
  {
    if (req.start_time().has_value() &&
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

  //============================================================================
  State::ResourceSchedule::const_iterator
  get_search_end(const ReservationRequest& req)
  {
    if (req.start_time().has_value() &&
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

  //============================================================================
  std::optional<State> advance_within_resource()
  {
    if (proceed_next_resource)
      return std::nullopt;

    State next_state(*start_state);

    auto [part_id, req_id] = *unassigned_iter;

    auto current_request = start_state->_queue->get_request_info(
      part_id,
      req_id)->request_options[request_index_iter];

    proceed_next_resource = false;

    while (insertion_point_iter !=
      start_state->_resource_schedules[current_request.resource_name()].end())
    {
      if (insertion_point_iter->second.actual_finish_time().has_value())
      {
        auto reservation = Reservation::make_reservation(
          insertion_point_iter->second.actual_finish_time().value(),
          current_request.resource_name(),
          current_request.duration(),
          current_request.finish_time()
        );

        bool found_slot = next_state.assign_reservation(
          part_id,
          req_id,
          reservation
        );
        insertion_point_iter++;
        if (found_slot)
        {
          return next_state;
        }
      }
      else
      {
        return std::nullopt;
      }
    }

    proceed_next_resource = true;

    if (
      current_request.start_time().has_value() &&
      current_request.start_time()->lower_bound().has_value())
    {
      auto reservation = Reservation::make_reservation(
        current_request.start_time().value().lower_bound().value(),
        current_request.resource_name(),
        current_request.duration(),
        current_request.finish_time()
      );
      bool found_slot = next_state.assign_reservation(
        part_id,
        req_id,
        reservation
      );
      if (found_slot)
      {
        return next_state;
      }
      else
      {
        return std::nullopt;
      }
    }
    else
    {
      auto reservation = Reservation::make_reservation(
        start_state->_current_time,
        current_request.resource_name(),
        current_request.duration(),
        current_request.finish_time()
      );

      bool found_slot = next_state.assign_reservation(
        part_id,
        req_id,
        reservation
      );
      if (found_slot)
      {
        return next_state;
      }
      else
      {
        return std::nullopt;
      }
    }
  }

  //============================================================================
  std::optional<State> advance_within_query()
  {
    auto state = advance_within_resource();
    if (state.has_value())
    {
      return state;
    }

    request_index_iter++;
    auto [part_id, req_id] = *unassigned_iter;
    auto num_alternative = start_state->_queue->get_request_info(
      part_id,
      req_id)->request_options.size();

    while (request_index_iter < num_alternative)
    {
      // Reset resource iterators
      auto curr_request = start_state->_queue->get_request_info(part_id, req_id)
        ->request_options[request_index_iter];
      insertion_point_iter = get_search_start(curr_request);
      insertion_point_end = get_search_end(curr_request);
      proceed_next_resource = false;
      auto state = advance_within_resource();
      if (state.has_value())
      {
        return state;
      }
      request_index_iter++;
    }

    return std::nullopt;
  }

  //============================================================================
  std::optional<State> advance_assignments()
  {
    auto state = advance_within_query();
    if (state.has_value())
    {
      return state;
    }
    unassigned_iter++;
    while (unassigned_iter != start_state->_unassigned.end())
    {
      auto [part_id, req_id] = *unassigned_iter;
      request_index_iter = 0;
      auto curr_request = start_state->_queue->get_request_info(part_id, req_id)
        ->request_options[request_index_iter];
      insertion_point_iter = get_search_start(curr_request);
      insertion_point_end = get_search_end(curr_request);
      proceed_next_resource = false;
      auto state = advance_within_query();
      if (state.has_value())
      {
        return state;
      }
    }
    return std::nullopt;
  }

  //============================================================================
  std::optional<State> advance_removals()
  {
    if (remove_resource_iter == start_state->_resource_schedules.end())
    {
      return std::nullopt;
    }
    while (remove_iter == remove_resource_iter->second.end())
    {
      remove_resource_iter++;
      if (remove_resource_iter == start_state->_resource_schedules.end())
      {
        return std::nullopt;
      }
      remove_iter = remove_resource_iter->second.begin();
    }

    State new_state(*start_state);
    new_state.unassign_reservation(
      remove_iter->second.reservation_id(),
      remove_iter->second.resource_name());
    remove_iter++;
    return new_state;
  }

  //============================================================================
  std::optional<State> next_state()
  {
    if (mode == PotentialActionMode::ASSIGN_ITEMS)
    {
      auto assignment = advance_assignments();
      if (assignment.has_value())
      {
        visiting_state = assignment;
        return assignment;
      }
      mode = PotentialActionMode::REMOVE_ITEMS;
    }
    auto removals = advance_removals();

    if (removals.has_value())
    {
      visiting_state = removals;
      return removals;
    }
    mode = PotentialActionMode::END;
    return std::nullopt;
  }

  //============================================================================
  NextStateGenerator& operator++()
  {
    visiting_state = next_state();
    return *this;
  }

  //============================================================================
  NextStateGenerator operator++(int)
  {
    NextStateGenerator r = *this;
    ++(*this);
    return r;
  }

  //============================================================================
  bool operator==(const NextStateGenerator& other)
  {
    if (mode == PotentialActionMode::END
      && other.mode == PotentialActionMode::END)
    {
      return true;
    }
    return false;
  }

  //============================================================================
  bool operator!=(const NextStateGenerator& other)
  {
    return !(*this == other);
  }
};

//==============================================================================
NextStateGenerator State::begin()
{
  NextStateGenerator gen;
  gen.mode = NextStateGenerator::PotentialActionMode::ASSIGN_ITEMS;
  gen.start_state = this;

  if (_unassigned.size() > 0)
  {
    auto request = _queue->get_request_info(
      _unassigned.begin()->first,
      _unassigned.begin()->second);

    if (!request.has_value())
    {
      // This should *never* happen
      throw std::runtime_error(std::string("Got nullopt for participant ") +
              std::to_string(_unassigned.begin()->first) +
              std::string(" request id #") +
              std::to_string(_unassigned.begin()->second));
    }

    auto current_request = request->request_options[0];

    gen.unassigned_iter = _unassigned.begin();
    gen.insertion_point_iter = gen.get_search_start(current_request);
    gen.insertion_point_end = gen.get_search_end(current_request);
  }
  gen.remove_resource_iter = _resource_schedules.begin();

  if (_resource_schedules.size() > 0)
  {
    gen.remove_iter = gen.remove_resource_iter->second.begin();
  }
  else if (_unassigned.size() == 0)
  {
    return end();
  }
  gen.next_state();
  return gen;
}

//==============================================================================
NextStateGenerator State::end()
{
  NextStateGenerator gen;
  gen.mode = NextStateGenerator::PotentialActionMode::END;
  return gen;
}
}
}
#endif