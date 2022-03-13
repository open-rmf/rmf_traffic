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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_DATABASE_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_DATABASE_HPP

#include <rmf_traffic/schedule/Database.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
void internal_register_participant(
  Database& database,
  ParticipantId id,
  ItineraryVersion last_known_version,
  ParticipantDescription description);

//==============================================================================
struct RouteStorageInfo
{
  RouteId route_id;
  StorageId storage_id;
  std::shared_ptr<const Route> route;
};

//==============================================================================
void set_participant_state(
  Database& database,
  ParticipantId participant,
  PlanId plan,
  std::vector<RouteStorageInfo> routes,
  StorageId storage_base,
  ItineraryVersion itinerary_version,
  std::vector<CheckpointId> progress,
  ProgressVersion progress_version);

//==============================================================================
void set_initial_fork_version(
  Database& database,
  Version version);

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_DATABASE_HPP
