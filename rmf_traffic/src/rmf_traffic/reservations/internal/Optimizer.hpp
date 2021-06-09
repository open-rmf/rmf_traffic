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
#include "priority_queue.hpp"

namespace rmf_traffic {
namespace reservations {

struct StateHash {
   size_t operator() (const State &state) const {
     return state.hash();
   }
};
class GreedyBestFirstSearchOptimizer
{
public:
  class CustomComparator
  {
    std::shared_ptr<Heuristic> _heuristic;
  public:
    CustomComparator(std::shared_ptr<Heuristic> heuristic):
      _heuristic(heuristic)
    {

    }
    bool operator() (const State& e1, const State& e2) const
    {
      return _heuristic->score(e1) < _heuristic->score(e2);
    }
  };
  class Solution
  {
  public:
    Solution(State& state,
      std::shared_ptr<Heuristic> heuristic):
      _last_solution(state),
      _pq(heuristic),
      _all_solutions(heuristic),
      _heuristic(heuristic)
    {
      _blacklist.insert(state);
      _visited.insert(state);
      _pq.push(state);
    }
    std::optional<State> next_solution()
    {
      _blacklist.insert(_last_solution);
      while(!_pq.empty())
      {
        auto state = _pq.top();
        _visited.insert(state);
        _pq.pop();
        for(auto next_state: state)
        {
          if(_visited.count(next_state) != 0)
            continue;

          if(_heuristic->score(next_state) == 0 &&
            _blacklist.count(next_state) == 0)
          {
            _last_solution = next_state;
            return next_state;
          }

          _pq.push(next_state);
          _all_solutions.push(next_state);
        }
      }
      while(!_all_solutions.empty())
      {
        auto res = _all_solutions.top();
        _last_solution = res;
        _all_solutions.pop();
        if(_blacklist.count(res) != 0)
          return res;
      }
      return std::nullopt;
    }
  private:
    std::unordered_set<State, StateHash> _visited;
    std::unordered_set<State, StateHash> _blacklist;
    std::priority_queue<State, std::vector<State>, CustomComparator> _pq;
    std::priority_queue<State, std::vector<State>, CustomComparator>
      _all_solutions;
    std::shared_ptr<Heuristic> _heuristic;
    State _last_solution;
  };
  GreedyBestFirstSearchOptimizer(std::shared_ptr<Heuristic> heuristic):
    _heuristic(heuristic)
  {
  }

  Solution optimize(State& state)
  {
    
  }
private:
  std::shared_ptr<Heuristic> _heuristic;
};
}
}
#endif