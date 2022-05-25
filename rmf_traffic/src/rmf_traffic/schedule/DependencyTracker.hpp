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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__DEPENDENCYTRACKER_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__DEPENDENCYTRACKER_HPP

#include "internal_Viewer.hpp"

#include <mutex>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
struct DependencyTracker
{
  using Shared =
    ItineraryViewer::DependencySubscription::Implementation::Shared;
  using DependencyPtr = std::weak_ptr<Shared>;
  using CheckpointDependencies =
    std::map<CheckpointId, std::vector<DependencyPtr>>;
  using RouteDependencies =
    std::unordered_map<RouteId, CheckpointDependencies>;
  using PlanDependencies =
    std::unordered_map<PlanId, RouteDependencies>;
  using TrafficDependencies =
    std::unordered_map<ParticipantId, PlanDependencies>;

  void add(Dependency dep, DependencyPtr shared);

  void reached(
    const ParticipantId participant,
    const PlanId plan,
    const std::vector<CheckpointId>& reached_checkpoints);

  void deprecate_dependencies_before(
    const ParticipantId participant,
    const PlanId plan);

  void deprecate_dependencies_on(const ParticipantId participant);

//private:
  std::mutex _mutex;
  TrafficDependencies _dependencies;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__DEPENDENCYTRACKER_HPP
