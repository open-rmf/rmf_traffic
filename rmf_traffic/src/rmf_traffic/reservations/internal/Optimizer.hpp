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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_OPTIMIZER_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_OPTIMIZER_HPP

#include "Heuristic.hpp"

namespace rmf_traffic {
namespace reservations {

struct StateHash {
   size_t operator() (const State &state) const {
     return state.hash();
   }
};
class Optimizer
{
public:
  class Solution
  {
  public:
    std::optional<State> next_solution()
    {
      
    }
  private:
    std::unordered_set<State> visited;
    State last_solution;
  };
  Optimizer(std::unique_ptr<Heuristic> heuristic):
    _heuristic(std::move(heuristic))
  {
  }

  Solution optimize(State& state)
  {
    
  }
private:
  std::unique_ptr<Heuristic> _heuristic;
};
}

#endif