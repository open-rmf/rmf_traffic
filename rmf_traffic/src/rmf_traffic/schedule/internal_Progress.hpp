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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PROGRESS_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PROGRESS_HPP

#include <rmf_traffic/schedule/Writer.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
struct Progress
{
  ProgressVersion version = 0;
  std::vector<CheckpointId> reached_checkpoints;

  void resize(std::size_t size);
  bool update(
    RouteId route,
    CheckpointId checkpoint,
    std::optional<ProgressVersion> new_version = std::nullopt);
};

//==============================================================================
/// Buffer the progress information of plans that haven't been received yet.
struct ProgressBuffer
{
public:

  /// Add progress to the buffer
  void buff(
    PlanId plan,
    RouteId route,
    CheckpointId checkpoint,
    std::optional<ProgressVersion> version = std::nullopt);

  /// Pull progress from the buffer and wipe out any buffered progress up to and
  /// including the one for this plan.
  Progress pull(PlanId plan, std::size_t itin_size);

private:
  std::unordered_map<PlanId, Progress> _buffer;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PROGRESS_HPP
