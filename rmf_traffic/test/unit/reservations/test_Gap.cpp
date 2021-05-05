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

#include <rmf_utils/catch.hpp>
#include "../../../src/rmf_traffic/reservations/planner/internal_Gap.hpp"

using namespace rmf_traffic::reservations;
using namespace rmf_traffic;
using namespace std::chrono_literals;

void debug_gap_map(std::map<Time, Gap>& gaps)
{
  for(auto &[time, gap]: gaps)
  {
    std::cout << "Gap starting at " <<
        time.time_since_epoch().count() <<std::endl;
    std::cout << "\tDuration: " << gap.time_gap.count();
    std::cout << "\tReservation pair: (" << gap.r1.value()
      << "," << gap.r2.value() << ")"<<std::endl;
    std::cout << "\tConflict Tables: " << std::endl;
    std::cout << "\t\tpush_back: ";
    for(auto duration: gap.conflict_table_push_back)
    {
      if(duration.has_value())
        std::cout << duration->count() << " ";
      else
        std::cout << "nil ";
    }
    std::cout << std::endl;
    std::cout << "\t\tbring_forward: ";
    for(auto duration: gap.conflict_table_bring_forward)
    {
      if(duration.has_value())
        std::cout << duration->count() << " ";
      else
        std::cout << "nil ";
    }
    std::cout << std::endl;
  }
}
SCENARIO("Given a schedule with 3 appointments with gaps")
{
  AbstractScheduleState::ResourceSchedule sched;
  
  auto now = std::chrono::steady_clock::now();
  now = now - now.time_since_epoch();

  Reservation res1 = Reservation::make_reservation(
    now+10min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  Reservation res2 = Reservation::make_reservation(
    now+40min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  Reservation res3 = Reservation::make_reservation(
    now+60min,
    "node_1",
    0,
    {20min},
    std::nullopt
  );

  sched.insert({res1.start_time(), res1});
  sched.insert({res2.start_time(), res2});
  sched.insert({res3.start_time(), res3});

  WHEN("Generating a gap map")
  {
    auto gap_map = generate_gap_map(sched);
    debug_gap_map(gap_map);
  }
}