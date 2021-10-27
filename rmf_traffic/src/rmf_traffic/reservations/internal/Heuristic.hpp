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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_HEURISTIC_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_HEURISTIC_HPP

#include "State.hpp"

namespace rmf_traffic {
namespace reservations {

class Heuristic
{
public:
  virtual float score(const State& state) = 0;

  virtual ~Heuristic() = default;
};

class PriorityBasedScorer : public Heuristic
{
public:
  PriorityBasedScorer()
  {
    // Do nothing
  }

  ~PriorityBasedScorer()
  {
    // Do nothing
  }

  float score(const State& state) override
  {
    // TODO: This is not really priority based planning.
    // Currently we first minimize unassigned priorities, then we
    // minimize the cost...
    // Cost itself is determined by the order of the alternatives presented.
    auto score = 0;
    for (auto [part_id, req_id]: state.unassigned())
    {
      // Incorporate priority
      score += state.requests()->get_request_info(part_id, req_id)->priority;
    }

    auto num_alternatives = state.requests()->num_alternatives();
    // Enforces ordering
    for (auto&[resource, schedule] : state.resource_schedule())
    {
      for (auto&[time, reservation]: schedule)
      {
        double index = state.request_ids()[reservation.reservation_id()].index;
        score += index / (double)num_alternatives;
      }
    }
    return score;
  }
};

}
}

#endif
