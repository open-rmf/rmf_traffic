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
#ifndef SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_GAP
#define SRC__RMF_TRAFFIC__RESERVATIONS__INTERNAL_GAP
#include <rmf_traffic/reservations/Database.hpp>
#include "../internal_State.hpp"
#include "../internal_ConstraintTracker.hpp"
#include <set>
#include <map>
#include <queue>

namespace rmf_traffic {
namespace reservations {

//==============================================================================
// A gap represents the timespan between to items in a resource schedule.
// Keeping track of gaps allows us to simplify certain plans when doing so
// This is used by the system to pick appropriate insertion points for
// reservations and to minimize statespace explosion.
class Gap
{
public:
  // The gap between reservations.
  std::optional<ReservationId> r1, r2;

  // The gap size
  Duration time_gap;

  // The i-th index reflects the time to i+1 conflicts. This gives the system
  // hints about where to perform insertions.
  // The duration is the time from the end of the gap to the next one.
  std::vector<std::optional<Duration>> conflict_table_push_back;
  std::vector<std::optional<Duration>> conflict_table_bring_forward;

  Gap(
    AbstractScheduleState::ResourceSchedule& sched,
    AbstractScheduleState::ResourceSchedule::const_iterator it
  )
  {
    auto it_next = std::next(it);
    std::cout << __FILE__ << ": " << __LINE__ <<std::endl;
    if(it_next == sched.end())
    {
      throw std::runtime_error("Reached end of schedule. Not a gap");
    }
    // Cannot be infinite reservation as it is not at the end of the schedule
    auto time_start = it->second.actual_finish_time().value();
    auto time_end = it_next->first;

    this->r1 = it->second.reservation_id();
    this->r2 = it_next->second.reservation_id();
    this->time_gap = time_end - time_start;
    nth_conflict_times_bring_forward(sched, it);
    nth_conflict_times_push_back(sched, it_next);
  }
private:
  void nth_conflict_times_bring_forward(
    AbstractScheduleState::ResourceSchedule& sched,
    AbstractScheduleState::ResourceSchedule::const_iterator iter)
  {

    int conflicts = 0;
    Duration dur{0};
    conflict_table_bring_forward.push_back(dur);
    Duration conflict_duration{0};
    while(true)
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
        conflict_table_bring_forward[conflict_table_bring_forward.size() - 1]
          = std::nullopt;
      }
      conflict_table_bring_forward.push_back({conflict_duration});
      iter = _prev_res;
    }
  }

  std::vector<std::optional<Duration>> nth_conflict_times_push_back(
    AbstractScheduleState::ResourceSchedule& sched,
    AbstractScheduleState::ResourceSchedule::const_iterator iter)
  {
    int conflicts = 0;
    Duration dur{0};
    conflict_table_push_back.push_back(dur);
    Duration conflict_duration{0};
    while(true)
    {
      if(iter ==sched.end())
      {
        break;
      }
      auto _next_res = std::next(iter);
      if(_next_res==sched.end())
      {
        break;
      }
      auto gap = _next_res->first - iter->second.actual_finish_time().value();

      //TODO: consider other constraints
      conflict_duration += gap;
      if(gap.count() == 0)
      {
        // There is no gap so it is not possible to have n conflicts
        conflict_table_push_back[conflict_table_push_back.size() - 1] = std::nullopt;
      }
      conflict_table_push_back.push_back({conflict_duration});

      iter = _next_res;
    }
    return conflict_table_push_back;
  }
};

const std::map<Time, Gap> generate_gap_map(
  AbstractScheduleState::ResourceSchedule& sched)
{
  std::map<Time, Gap> gaps;
  int index = 0;
  for(auto it = sched.begin(); it !=sched.end(); it++)
  {
    auto it_next = std::next(it);
    if(it_next == sched.end())
    {
      break;
    }
    // Cannot be infinite reservation as it is not at the end of the schedule
    Gap gap(sched, it);
    std::cout << "index" << index <<std::endl;
    gaps.insert({it->second.actual_finish_time().value(), gap});
  }
  return gaps;
}

}
}
#endif
