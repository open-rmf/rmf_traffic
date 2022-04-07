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

#include <rmf_traffic/schedule/Patch.hpp>
#include "../detail/internal_bidirectional_iterator.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Patch::Participant::Implementation
{
public:

  ParticipantId id;
  ItineraryVersion itinerary_version;
  Change::Erase erasures;
  std::vector<Change::Delay> delays;
  Change::Add additions;
  std::optional<Change::Progress> progress;
};

//==============================================================================
Patch::Participant::Participant(
  ParticipantId id,
  ItineraryVersion itinerary_version,
  Change::Erase erasures,
  std::vector<Change::Delay> delays,
  Change::Add additions,
  std::optional<Change::Progress> progress)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        id,
        itinerary_version,
        std::move(erasures),
        std::move(delays),
        std::move(additions),
        std::move(progress)
      }))
{
  // Do nothing
}

//==============================================================================
ParticipantId Patch::Participant::participant_id() const
{
  return _pimpl->id;
}

//==============================================================================
ItineraryVersion Patch::Participant::itinerary_version() const
{
  return _pimpl->itinerary_version;
}

//==============================================================================
const Change::Erase& Patch::Participant::erasures() const
{
  return _pimpl->erasures;
}

//==============================================================================
const std::vector<Change::Delay>& Patch::Participant::delays() const
{
  return _pimpl->delays;
}

//==============================================================================
const Change::Add& Patch::Participant::additions() const
{
  return _pimpl->additions;
}

//==============================================================================
const std::optional<Change::Progress>& Patch::Participant::progress() const
{
  return _pimpl->progress;
}

//==============================================================================
class Patch::Implementation
{
public:

  std::vector<Participant> changes;

  std::optional<Change::Cull> cull;
  std::optional<Version> base_version;
  Version latest_version;

};

//==============================================================================
class Patch::IterImpl
{
public:

  std::vector<Participant>::const_iterator iter;

};

//==============================================================================
Patch::Patch(std::vector<Participant> changes,
  rmf_utils::optional<Change::Cull> cull,
  std::optional<Version> base_version,
  Version latest_version)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(changes),
        cull,
        base_version,
        latest_version
      }))
{
  // Do nothing
}

//==============================================================================
auto Patch::begin() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.begin()});
}

//==============================================================================
auto Patch::end() const -> const_iterator
{
  return const_iterator(IterImpl{_pimpl->changes.end()});
}

//==============================================================================
std::size_t Patch::size() const
{
  return _pimpl->changes.size();
}

//==============================================================================
const Change::Cull* Patch::cull() const
{
  if (_pimpl->cull)
    return &_pimpl->cull.value();

  return nullptr;
}

//==============================================================================
std::optional<Version> Patch::base_version() const
{
  return _pimpl->base_version;
}

//==============================================================================
Version Patch::latest_version() const
{
  return _pimpl->latest_version;
}

} // namespace schedule


namespace detail {

template class bidirectional_iterator<
    const schedule::Patch::Participant,
    schedule::Patch::IterImpl,
    schedule::Patch
>;

} // namespace detail
} // namespace rmf_traffic
