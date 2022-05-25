/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ViewerInternal.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"
#include "../DetectConflictInternal.hpp"

#include <rmf_traffic/schedule/Viewer.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include "debug_Viewer.hpp"
#include "internal_Viewer.hpp"

#include <algorithm>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Viewer::View::IterImpl
{
public:

  std::vector<Element>::const_iterator iter;

};

//==============================================================================
auto Viewer::View::begin() const -> const_iterator
{
  return const_iterator{IterImpl{_pimpl->elements.begin()}};
}

//==============================================================================
auto Viewer::View::end() const -> const_iterator
{
  return const_iterator{IterImpl{_pimpl->elements.end()}};
}

//==============================================================================
std::size_t Viewer::View::size() const
{
  return _pimpl->elements.size();
}

//==============================================================================
void ItineraryViewer::DependencySubscription::Implementation::Shared::reach()
{
  reached = true;
  on_reached();
}

//==============================================================================
void
ItineraryViewer::DependencySubscription::Implementation::Shared::deprecate()
{
  deprecated = true;
  on_deprecated();
}

//==============================================================================
ItineraryViewer::DependencySubscription
ItineraryViewer::DependencySubscription::Implementation::make(
  Dependency dep,
  std::function<void()> on_reached,
  std::function<void()> on_deprecated)
{
  DependencySubscription output;
  output._pimpl = rmf_utils::make_unique_impl<Implementation>();
  output._pimpl->shared = std::make_shared<Shared>(
    Shared{dep, std::move(on_reached), std::move(on_deprecated)});

  return output;
}
//==============================================================================
auto ItineraryViewer::DependencySubscription::Implementation::get_shared(
  const DependencySubscription& sub) -> std::shared_ptr<Shared>
{
  return sub._pimpl->shared;
}

//==============================================================================
bool ItineraryViewer::DependencySubscription::reached() const
{
  return _pimpl->shared->reached;
}

//==============================================================================
bool ItineraryViewer::DependencySubscription::deprecated() const
{
  return _pimpl->shared->deprecated;
}

//==============================================================================
bool ItineraryViewer::DependencySubscription::finished() const
{
  return reached() || deprecated();
}

//==============================================================================
Dependency ItineraryViewer::DependencySubscription::dependency() const
{
  return _pimpl->shared->dependency;
}

//==============================================================================
ItineraryViewer::DependencySubscription::DependencySubscription()
{
  // Do nothing
}

} // namespace schedule


namespace detail {

template class bidirectional_iterator<
    const schedule::Viewer::View::Element,
    schedule::Viewer::View::IterImpl,
    schedule::Viewer::View
>;

} // namespace detail
} // namespace rmf_traffic
