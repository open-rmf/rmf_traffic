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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_VIEWER_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_VIEWER_HPP

#include <rmf_traffic/schedule/Viewer.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class ItineraryViewer::DependencySubscription::Implementation
{
public:
  struct Shared
  {
    Dependency dependency;
    std::function<void()> on_reached;
    std::function<void()> on_deprecated;
    bool reached = false;
    bool deprecated = false;

    void reach();
    void deprecate();
  };

  std::shared_ptr<Shared> shared;

  static DependencySubscription make(
    Dependency dep,
    std::function<void()> on_reached,
    std::function<void()> on_deprecated);

  static std::shared_ptr<Shared> get_shared(
    const DependencySubscription& sub);
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_VIEWER_HPP
