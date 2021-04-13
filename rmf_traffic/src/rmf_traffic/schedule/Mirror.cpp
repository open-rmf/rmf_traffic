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

#include <rmf_traffic/schedule/Mirror.hpp>

#include "ChangeInternal.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"
#include "internal_Snapshot.hpp"
#include "internal_Database.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Mirror::Implementation
{
public:

  using RouteEntry = BaseRouteEntry;
  using ConstRouteEntryPtr = std::shared_ptr<const RouteEntry>;

  struct RouteStorage
  {
    ConstRouteEntryPtr entry;
    std::shared_ptr<void> timeline_handle;
  };

  Timeline<const RouteEntry> timeline;

  struct ParticipantState
  {
    std::unordered_map<RouteId, RouteStorage> storage;
    std::shared_ptr<const ParticipantDescription> description;
    ItineraryVersion itinerary_version;
  };

  // This violates the single-source-of-truth principle, but it helps make it
  // more efficient to create snapshots
  using ParticipantDescriptions =
    std::unordered_map<ParticipantId,
      std::shared_ptr<const ParticipantDescription>
    >;
  ParticipantDescriptions descriptions;

  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  std::unordered_set<ParticipantId> participant_ids;

  Version latest_version = 0;

  static void erase_routes(
    const ParticipantId participant,
    ParticipantState& state,
    const Change::Erase& erase)
  {
    for (const RouteId id : erase.ids())
    {
      const auto r_it = state.storage.find(id);
      if (r_it == state.storage.end())
      {
        std::cerr << "[Mirror::update] Erasing unrecognized route [" << id
                  << "] for participant [" << participant << "]" << std::endl;
        continue;
      }

      state.storage.erase(r_it);
    }
  }

  void apply_delay(
    ParticipantState& state,
    const Change::Delay& delay)
  {
    for (auto& s : state.storage)
    {
      RouteStorage& entry_storage = s.second;
      assert(entry_storage.entry);
      assert(entry_storage.entry->route);
      auto delayed = schedule::apply_delay(
        entry_storage.entry->route->trajectory(), delay.duration());

      if (!delayed)
        continue;

      auto new_route = std::make_shared<Route>(
        entry_storage.entry->route->map(), std::move(*delayed));

      // We create a new entry because
      auto new_entry = std::make_shared<RouteEntry>(*entry_storage.entry);
      new_entry->route = std::move(new_route);
      entry_storage.entry = new_entry;
      entry_storage.timeline_handle = timeline.insert(new_entry);
    }
  }

  void add_route(
    const ParticipantId participant,
    ParticipantState& state,
    const RouteId route_id,
    const ConstRoutePtr& route)
  {
    auto insertion = state.storage.insert({route_id, RouteStorage()});
    const bool inserted = insertion.second;
    if (!inserted)
    {
      std::cerr << "[Mirror::update] Inserting a route [" << route_id
                << "] which already exists for participant [" << participant
                << "]" << std::endl;
      // NOTE(MXG): We will continue anyway. The new route will simply
      // overwrite the old one.
    }

    auto& entry_storage = insertion.first->second;
    entry_storage.entry = std::make_shared<RouteEntry>(
      RouteEntry{
        std::move(route),
        participant,
        route_id,
        state.description
      });

    entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
  }

  void add_routes(
    const ParticipantId participant,
    ParticipantState& state,
    const Change::Add& add)
  {
    for (const auto& item : add.items())
    {
      add_route(
        participant,
        state,
        item.id,
        std::make_shared<const Route>(*item.route));
    }
  }
};

namespace {
//==============================================================================
class MirrorViewRelevanceInspector
  : public TimelineInspector<Mirror::Implementation::RouteEntry>
{
public:

  using RouteEntry = Mirror::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry);
    assert(entry->route);
    if (relevant(*entry))
    {
      routes.emplace_back(
        Storage{
          entry->participant,
          entry->route_id,
          entry->route,
          entry->description
        });
    }
  }

};

//==============================================================================
class MirrorCullRelevanceInspector
  : public TimelineInspector<Mirror::Implementation::RouteEntry>
{
public:

  using RouteEntry = Mirror::Implementation::RouteEntry;
  struct Info
  {
    ParticipantId participant;
    RouteId route_id;
  };

  std::vector<Info> info;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry);
    assert(entry->route);
    if (relevant(*entry))
      info.emplace_back(Info{entry->participant, entry->route_id});
  }

};

} // anonymous namespace

//==============================================================================
Viewer::View Mirror::query(const Query& parameters) const
{
  return query(parameters.spacetime(), parameters.participants());
}

//==============================================================================
Viewer::View Mirror::query(
  const Query::Spacetime& spacetime,
  const Query::Participants& participants) const
{
  MirrorViewRelevanceInspector inspector;
  _pimpl->timeline.inspect(spacetime, participants, inspector);
  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Mirror::participant_ids() const
{
  return _pimpl->participant_ids;
}

//==============================================================================
std::shared_ptr<const ParticipantDescription> Mirror::get_participant(
  std::size_t participant_id) const
{
  const auto p = _pimpl->descriptions.find(participant_id);
  if (p == _pimpl->descriptions.end())
    return nullptr;

  return p->second;
}

//==============================================================================
rmf_utils::optional<Itinerary> Mirror::get_itinerary(
  std::size_t participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return rmf_utils::nullopt;

  const auto& state = p->second;
  Itinerary itinerary;
  itinerary.reserve(state.storage.size());
  for (const auto& s : state.storage)
    itinerary.push_back(s.second.entry->route);

  return itinerary;
}

//==============================================================================
Version Mirror::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
std::shared_ptr<const Snapshot> Mirror::snapshot() const
{
  using SnapshotType =
    SnapshotImplementation<Implementation::RouteEntry,
      MirrorViewRelevanceInspector
    >;

  return std::make_shared<SnapshotType>(
    _pimpl->timeline.snapshot(nullptr),
    _pimpl->participant_ids,
    _pimpl->descriptions,
    _pimpl->latest_version);
}

//==============================================================================
Mirror::Mirror()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
void Mirror::update_participants_info(
  const ParticipantDescriptionsMap& participants)
{
  // First remove any participants that are no longer around.
  // We create a removed_ids list to start, because otherwise we would be
  // iterating through _pimpl->states while also erasing elements from it, which
  // results in undefined behavior and may cause a segmentation fault.
  std::unordered_set<ParticipantId> removed_ids;
  for ([[maybe_unused]] const auto& [id, state]: _pimpl->states)
  {
    const auto p_it = participants.find(id);
    if (p_it == participants.end())
      removed_ids.insert(id);
  }

  for (const auto id : removed_ids)
  {
    // This participant is not in the updated list of participants, so remove
    // the mirror's copy of it
    _pimpl->states.erase(id);
    _pimpl->descriptions.erase(id);
    _pimpl->participant_ids.erase(id);
  }

  // Next add-or-update all participants that are currently present
  for (const auto& [id, description]: participants)
  {
    const auto p_it = _pimpl->states.find(id);
    if (p_it == _pimpl->states.end())
    {
      // This is a new participant. We need to add a new state entry for it.
      // We do not add a state for it yet because we know nothing about its
      // routes or itinerary version.
      const auto description_ptr =
        std::make_shared<const ParticipantDescription>(description);
      _pimpl->descriptions.insert({id, description_ptr});
      _pimpl->participant_ids.insert(id);
    }
    else
    {
      // This is an existing participant. We need to overwrite the description
      // and then replace all the timeline entries with new ones that contain
      // the new description.
      p_it->second.description =
        std::make_shared<const ParticipantDescription>(description);

      const auto participant_id = p_it->first;
      auto& state = p_it->second;

      // Make a copy of the routes before we clear them out
      const auto old_storage = state.storage;

      // Clear out the state's copy of the route information
      p_it->second.storage.clear();

      // Insert new routes that are equivalent to the old ones, but which have
      // the updated description.
      for (const auto& [route_id, route_storage] : old_storage)
      {
        const auto& route = route_storage.entry->route;
        _pimpl->add_route(participant_id, state, route_id, route);
      }
    }
  }
}

//==============================================================================
bool Mirror::update(const Patch& patch)
{
  if (_pimpl->latest_version >= patch.latest_version())
  {
    // This patch is older than or equal to the information this mirror already
    // has, so it can safely be ignored.
    return true;
  }

  if (patch.base_version().has_value())
  {
    if (*patch.base_version() != _pimpl->latest_version)
      return false;
  }
  else
  {
    // If the patch is assuming that we're starting from a blank slate, then
    // we'll simply erase all the itineraries we currently have and apply the
    // patch on top of a blank slate.
    for (auto& [_, state] : _pimpl->states)
      state.storage.clear();
  }

  for (const auto& p : patch)
  {
    const ParticipantId participant = p.participant_id();

    // Check if the mirror knows about this participant yet, and insert a blank
    // participant state if it does not.
    const auto insertion = _pimpl->states.insert({participant, {}});
    const auto p_it = insertion.first;
    Implementation::ParticipantState& state = p_it->second;
    state.itinerary_version = p.itinerary_version();
    const auto newly_inserted = insertion.second;
    if (newly_inserted)
    {
      // Unknown participant. We will create an empty description for it, unless
      // the description already exists
      const auto d_it = _pimpl->descriptions.insert({participant, {}}).first;
      state.description = d_it->second;
      _pimpl->participant_ids.insert(participant);
    }

    Implementation::erase_routes(participant, state, p.erasures());

    for (const auto& delay : p.delays())
      _pimpl->apply_delay(state, delay);

    _pimpl->add_routes(participant, state, p.additions());
  }

  if (const Change::Cull* cull = patch.cull())
  {
    const Time time = cull->time();
    Query query = query_all();
    query.spacetime().query_timespan().set_upper_time_bound(time);

    MirrorCullRelevanceInspector inspector;
    _pimpl->timeline.inspect(
      query.spacetime(), query.participants(), inspector);

    for (const auto& route : inspector.info)
    {
      auto p_it = _pimpl->states.find(route.participant);
      assert(p_it != _pimpl->states.end());
      if (p_it == _pimpl->states.end())
      {
        std::cerr << "[Mirror::update] Non-existent participant ["
                  << route.participant << "] in timeline entry" << std::endl;
        continue;
      }

      p_it->second.storage.erase(route.route_id);
    }
  }

  _pimpl->latest_version = patch.latest_version();
  return true;
}

//==============================================================================
Database Mirror::fork() const
{
  Database output;

  for (const auto& [id, description] : _pimpl->descriptions)
    internal_register_participant(output, id, *description);

  for (const auto& [participant, state] : _pimpl->states)
  {
    Writer::Input input;
    input.reserve(state.storage.size());
    for (const auto& [route_id, route] : state.storage)
      input.emplace_back(Writer::Item{route_id, route.entry->route});

    output.set(participant, input, state.itinerary_version);
  }

  set_initial_fork_version(output, _pimpl->latest_version);

  return output;
}

} // schedule
} // rmf_traffic
