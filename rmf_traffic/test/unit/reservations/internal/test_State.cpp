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

#include "../../../../src/rmf_traffic/reservations/internal/State.hpp"

#include <rmf_utils/catch.hpp>

using namespace rmf_traffic;
using namespace rmf_traffic::reservations;
using namespace std::chrono_literals;

SCENARIO("A few reservations in a state")
{
    auto queue = std::make_shared<RequestQueue>();
    State start_state(queue);

    auto now = std::chrono::steady_clock::now();
    now -= now.time_since_epoch();

    auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
            now,
            now+10s
        ),
        {10s}
    );

    auto request1 = std::vector{request1_alt1};
    queue->enqueue_reservation(0, 0, 1, request1);
    start_state = start_state.add_request(0, 0);

    std::cout << "Start State:" <<std::endl;
    start_state.debug_state();

    for(auto next_state: start_state)
    {
        std::cout << "Next State:" <<std::endl;
        next_state.debug_state();
    }
}