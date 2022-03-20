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

#include <rmf_utils/Modular.hpp>

#include "internal_Progress.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
void Progress::resize(std::size_t size)
{
  if (size > reached_checkpoints.size())
    reached_checkpoints.resize(size, 0);
}

//==============================================================================
bool Progress::update(
  const RouteId route,
  const CheckpointId checkpoint,
  const std::optional<ProgressVersion> new_version)
{
  if (reached_checkpoints.size() < route + 1)
    resize(route+1);

  if (reached_checkpoints[route] < checkpoint)
  {
    if (!new_version.has_value())
      ++version;
    else if (version < *new_version)
      version = *new_version;

    reached_checkpoints[route] = checkpoint;
    return true;
  }

  return false;
}

//==============================================================================
void ProgressBuffer::buff(
  PlanId plan,
  RouteId route,
  CheckpointId checkpoint,
  std::optional<ProgressVersion> new_version)
{
  _buffer[plan].update(route, checkpoint, new_version);
}

//==============================================================================
auto ProgressBuffer::pull(PlanId plan, std::size_t itin_size) -> Progress
{
  Progress new_progress = _buffer[plan];
  new_progress.resize(itin_size);

  auto it = _buffer.begin();
  while (it != _buffer.end())
  {
    const auto check_it = it;
    ++it;
    if (rmf_utils::modular(check_it->first).less_than_or_equal(plan))
      _buffer.erase(check_it);
  }

  return new_progress;
}

} // namespace schedule
} // namespace rmf_traffic
