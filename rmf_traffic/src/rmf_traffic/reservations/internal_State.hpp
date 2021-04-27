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
#ifndef SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE
#define SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_STATE
#include <rmf_traffic/reservations/Database.hpp>
#include <set>
#include <map>

#include <iostream>

namespace rmf_traffic {
namespace reservations {

//=============================================================================
// An AbstractScheduleState represents a schedule of reservations.
// Operations on this class can be performed by the planner or the current
// reservation store. Note: the schedule state may contain inconsitent
// reservations. Consistency is not the aim of the schedule state, rather
// the planner is responsible for it.
class AbstractScheduleState
{
public:
  using ResourceSchedule = std::map<rmf_traffic::Time, Reservation>;

  //virtual const std::vector<std::string> get_all_resources() const = 0;

  virtual const ResourceSchedule get_schedule(std::string resource_name) const
    = 0;

  virtual bool add_reservation(Reservation& res) = 0;

  virtual bool update_reservation(Reservation& res) = 0;

  virtual bool cancel_reservation(ReservationId id) = 0;

  virtual std::optional<Reservation>
    get_reservation_by_id(ReservationId res) const = 0;

};

//=============================================================================
// Stores the current schedule state.
// This class performs no checks on whether the schedule 
class CurrentScheduleState: public AbstractScheduleState
{
public:
  //===========================================================================
  // Storage Classes - These classes store the actual reservation
  // and the requests
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules;
  using ReservationMapping = std::unordered_map<ReservationId,
    std::pair<std::string, Time>>;
  ReservationMapping _reservation_mapping;

  const ResourceSchedule get_schedule(std::string name) const override
  {
    return _resource_schedules.at(name);
  }

  bool add_reservation(Reservation& reservation) override
  {
    const auto resource = reservation.resource_name();
    const auto reservation_id = reservation.reservation_id();

    if(_reservation_mapping.count(reservation_id) > 0)
      return false;

    _resource_schedules[resource]
        .insert({reservation.start_time(), reservation});
    _reservation_mapping[reservation_id] = {resource, reservation.start_time()};
    return true;
  }

  bool update_reservation(Reservation& reservation) override
  {
    const auto resource = reservation.resource_name();
    const auto reservation_id = reservation.reservation_id();

    if(_reservation_mapping.count(reservation_id) == 0)
      return false;

    cancel_reservation(reservation.reservation_id());
    auto [_, res] = _resource_schedules[resource].insert(
        {reservation.start_time(), reservation});

    if(!res)
      return false;

    _reservation_mapping[reservation_id] = {resource, reservation.start_time()};
    return true;
  }

  bool cancel_reservation(ReservationId reservation_id) override
  {
    if(_reservation_mapping.count(reservation_id) == 0)
      return false;

    auto [resource_name, start_time] = _reservation_mapping[reservation_id];
    _resource_schedules[resource_name].erase(start_time);
    _reservation_mapping.erase(reservation_id);
    return true;
  }

  std::optional<Reservation>
    get_reservation_by_id(ReservationId reservation_id) const override
  {
    auto mapping = _reservation_mapping.find(reservation_id);
    if(mapping == _reservation_mapping.end())
    {
      return std::nullopt;
    }

    auto sched = _resource_schedules.find(mapping->second.first);
    if(sched == _resource_schedules.end())
    {
      return std::nullopt;
    }
    auto it =
        sched->second.find(mapping->second.second);

    if(it == sched->second.end() ||
     it->first != mapping->second.second)
     return std::nullopt;

    return it->second;
  }
};

//==============================================================================
/// \brief This class is use by the planner to generate plans or create
/// patches in the schedule. It contains changes performed on a.
/// Currently this class implements a very inefficient copy-on-write protocol
/// which copies the *entire* ReservationSchedule for a given resource.
/// TODO: This class should eventually be rewritten to prevent unessecary
/// copying.
class SchedulePatch: public AbstractScheduleState
{
public:
  std::shared_ptr<const AbstractScheduleState> _parent;
  using ResourceSchedules = std::unordered_map<std::string, ResourceSchedule>;
  ResourceSchedules _resource_schedules_overlay;
  using ReservationMapping = std::unordered_map<ReservationId,
    std::pair<std::string, Time>>;
  ReservationMapping _reservation_mapping_overlay;
  std::unordered_set<ReservationId> cancelled;

  SchedulePatch(std::shared_ptr<const AbstractScheduleState> parent) :
    _parent(parent)
  {
    //Do Nothing.
  }

  const ResourceSchedule get_schedule(std::string resource) const override
  {
    auto sched_iter = _resource_schedules_overlay.find(resource);
    if(sched_iter == _resource_schedules_overlay.end())
        return _parent->get_schedule(resource);
    return sched_iter->second;
  }

  bool check_if_in_cache(std::string resource) const
  {
    auto sched_iter = _resource_schedules_overlay.find(resource);
    return sched_iter == _resource_schedules_overlay.end();
  }

  bool check_if_in_cache(ReservationId res) const
  {
    return _reservation_mapping_overlay.count(res) > 0 ||
      cancelled.count(res) > 0;
  }

  void cache_resource(std::string resource)
  {
    try
    {
      auto sched = _parent->get_schedule(resource);
      _resource_schedules_overlay[resource] = ResourceSchedule(sched);
      for(auto &[time, reservation] : sched)
      {
        _reservation_mapping_overlay[reservation.reservation_id()] =
          {resource, time};
      }
    }
    catch(std::out_of_range &e)
    {
      // Do nothing, resource doesn't exist in parent. Lazily create it.
    }
  }

  bool add_reservation(Reservation& reservation) override
  {
    const auto resource = reservation.resource_name();
    const auto reservation_id = reservation.reservation_id();

    if(check_if_in_cache(resource))
    {
       cache_resource(resource);
    }
    _resource_schedules_overlay[resource]
      .insert({reservation.start_time(), reservation});
    _reservation_mapping_overlay[reservation_id] =
      {resource, reservation.start_time()};
    return true;
  }

  bool update_reservation(Reservation& reservation) override
  {
    auto original_res =
        _parent->get_reservation_by_id(reservation.reservation_id());
    if(!original_res.has_value())
      return false;

    if(check_if_in_cache(original_res->resource_name()))
    {
      cache_resource(original_res->resource_name());
    }

    if(check_if_in_cache(reservation.resource_name()))
    {
      cache_resource(reservation.resource_name());
    }

    _resource_schedules_overlay[original_res->resource_name()]
    
    _reservation_mapping_overlay[reservation.reservation_id()] =
      {reservation.resource_name(), reservation.start_time()};
    return true;
  }

  bool cancel_reservation(ReservationId res) override
  {
    if(!check_if_in_cache(res))
    {
      auto reservation = _parent->get_reservation_by_id(res);
      if(!reservation.has_value())
      {
        return false;
      }
      cache_resource(reservation->resource_name());
    }

    auto &[resource_name, start_time] = _reservation_mapping_overlay[res];
    _resource_schedules_overlay[resource_name].erase(start_time);
    _reservation_mapping_overlay.erase(res);
    cancelled.insert(res);
    return true;
  }

  std::optional<Reservation>
    get_reservation_by_id(ReservationId reservation_id) const override
  {
    if(check_if_in_cache(reservation_id))
    {
      if(_reservation_mapping_overlay.count(reservation_id) == 0)
      {
        return std::nullopt;// reservation has been cancelled.
      }
      auto &[resource, time] = _reservation_mapping_overlay.at(reservation_id);
      try
      {
        return _resource_schedules_overlay.at(resource).at(time);
      }
      catch(std::out_of_range &e)
      {
        return std::nullopt;
      }
    }
    else
    {
      return _parent->get_reservation_by_id(reservation_id);
    }
  }
};


}
}

#endif
