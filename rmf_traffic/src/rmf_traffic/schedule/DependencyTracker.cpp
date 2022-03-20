/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "DependencyTracker.hpp"

#include <rmf_utils/Modular.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
void DependencyTracker::add(Dependency dep, DependencyPtr shared)
{
  std::lock_guard<std::mutex> lock(_mutex);
  _dependencies[dep.on_participant][dep.on_plan]
    [dep.on_route][dep.on_checkpoint].push_back(std::move(shared));
}

//==============================================================================
void DependencyTracker::reached(
  const ParticipantId participant,
  const PlanId plan,
  const std::vector<CheckpointId>& reached_checkpoints)
{
  std::lock_guard<std::mutex> lock(_mutex);
  auto& all_plan_deps = _dependencies[participant];
  const auto r_it = all_plan_deps.find(plan);
  if (r_it != all_plan_deps.end())
  {
    for (auto& [r, checkpoints] : r_it->second)
    {
      if (r >= reached_checkpoints.size())
        continue;

      auto c_it = checkpoints.begin();
      const auto c_end = checkpoints.upper_bound(reached_checkpoints[r]);
      while (c_it != c_end)
      {
        const auto reached_it = c_it;
        ++c_it;

        for (const auto& w : reached_it->second)
        {
          if (const auto dep = w.lock())
            dep->reach();
        }

        checkpoints.erase(reached_it);
      }
    }
  }
}

//==============================================================================
void DependencyTracker::deprecate_dependencies_before(
  const ParticipantId participant,
  const PlanId plan)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto d_it = _dependencies.find(participant);
  if (d_it != _dependencies.end())
  {
    auto p_it = d_it->second.begin();
    while (p_it != d_it->second.end())
    {
      const auto check_it = p_it;
      ++p_it;

      if (rmf_utils::modular(check_it->first).less_than(plan))
      {
        for (const auto& [_, r] : check_it->second)
        {
          for (const auto& [_, c] : r)
          {
            for (const auto& w : c)
            {
              if (const auto dep = w.lock())
              {
                dep->deprecate();
              }
            }
          }
        }

        d_it->second.erase(check_it);
      }
    }
  }
}

//==============================================================================
void DependencyTracker::deprecate_dependencies_on(
  const ParticipantId participant)
{
  std::lock_guard<std::mutex> lock(_mutex);
  const auto p_it = _dependencies.find(participant);
  if (p_it != _dependencies.end())
  {
    for (const auto& [_, r] : p_it->second)
    {
      for (const auto& [_, c] : r)
      {
        for (const auto& [_, deps] : c)
        {
          for (const auto& w : deps)
          {
            if (const auto dep = w.lock())
              dep->deprecate();
          }
        }
      }
    }

    _dependencies.erase(p_it);
  }
}

} // namespace schedule
} // namespace rmf_traffic
