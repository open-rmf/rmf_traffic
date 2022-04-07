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

#ifndef RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP
#define RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP

#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/schedule/Version.hpp>

#include <vector>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
using ItineraryVersion = uint64_t;

//==============================================================================
using Itinerary = std::vector<Route>;

//==============================================================================
using ItineraryView = std::vector<std::shared_ptr<const Route>>;

//==============================================================================
using ProgressVersion = uint64_t;

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__ITINERARY_HPP
