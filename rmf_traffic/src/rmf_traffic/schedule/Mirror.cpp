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

#include <rmf_utils/Modular.hpp>

#include "ChangeInternal.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"
#include "internal_Snapshot.hpp"
#include "internal_Database.hpp"
#include "internal_Progress.hpp"
#include "DependencyTracker.hpp"

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
    std::unordered_map<StorageId, RouteStorage> storage;
    std::shared_ptr<const ParticipantDescription> description;
    ItineraryVersion itinerary_version;
    PlanId current_plan_id = std::numeric_limits<PlanId>::max();
    std::optional<StorageId> highest_storage;
    Progress progress;
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

  mutable DependencyTracker dependencies;

  static void erase_routes(
    const ParticipantId participant,
    ParticipantState& state,
    const Change::Erase& erase)
  {
    for (const StorageId id : erase.ids())
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
      if (entry_storage.entry->route->trajectory().empty())
        continue;

      auto new_route = std::make_shared<Route>(*entry_storage.entry->route);
      new_route->trajectory().front().adjust_times(delay.duration());

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
    const StorageId storage_id,
    const ConstRoutePtr& route)
  {
    auto insertion = state.storage.insert({storage_id, RouteStorage()});
    const bool inserted = insertion.second;
    if (!inserted)
    {
      std::cerr << "[Mirror::update] Inserting a route at storage_id ["
                << storage_id << "] which is already used for participant ["
                << participant << "]" << std::endl;
      // NOTE(MXG): We will continue anyway. The new route will simply
      // overwrite the old one.
    }

    auto& entry_storage = insertion.first->second;
    entry_storage.entry = std::make_shared<RouteEntry>(
      RouteEntry{
        std::move(route),
        participant,
        state.current_plan_id,
        route_id,
        storage_id,
        state.description
      });

    entry_storage.timeline_handle = timeline.insert(entry_storage.entry);

    if (!state.highest_storage.has_value())
      state.highest_storage = storage_id;
    else if (rmf_utils::modular(*state.highest_storage).less_than(storage_id))
      state.highest_storage = storage_id;
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
        item.route_id,
        item.storage_id,
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
          entry->plan_id,
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
    StorageId storage_id;
  };

  std::vector<Info> info;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    assert(entry);
    assert(entry->route);
    if (relevant(*entry))
      info.emplace_back(Info{entry->participant, entry->storage_id});
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
Version Mirror::latest_version() const
{
  return _pimpl->latest_version;
}

//==============================================================================
std::optional<ItineraryView> Mirror::get_itinerary(
  const std::size_t participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
  {
    // If we don't have a state, it's possible that we have a description for
    // this participant but never received a state. In that case we should
    // return an empty itinerary, not a nullopt.
    if (_pimpl->participant_ids.count(participant_id) > 0)
      return ItineraryView{};

    return std::nullopt;
  }

  const auto& state = p->second;
  ItineraryView itinerary;
  itinerary.reserve(state.storage.size());
  for (const auto& s : state.storage)
    itinerary.push_back(s.second.entry->route);

  return itinerary;
}

//==============================================================================
std::optional<PlanId> Mirror::get_current_plan_id(
  const std::size_t participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return std::nullopt;

  return p->second.current_plan_id;
}

//==============================================================================
const std::vector<CheckpointId>* Mirror::get_current_progress(
  ParticipantId participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return nullptr;

  return &p->second.progress.reached_checkpoints;
}

//==============================================================================
auto Mirror::watch_dependency(
  Dependency dep,
  std::function<void()> on_reached,
  std::function<void()> on_deprecated) const -> DependencySubscription
{
  auto subscription = DependencySubscription::Implementation::make(
    dep, std::move(on_reached), std::move(on_deprecated));

  auto shared =
    DependencySubscription::Implementation::get_shared(subscription);

  const auto p_it = _pimpl->states.find(dep.on_participant);
  if (p_it == _pimpl->states.end())
  {
    shared->deprecate();
    return subscription;
  }

  const auto& state = p_it->second;
  if (rmf_utils::modular(dep.on_plan).less_than(state.current_plan_id))
  {
    shared->deprecate();
    return subscription;
  }

  if (state.current_plan_id == dep.on_plan)
  {
    if (dep.on_route < state.progress.reached_checkpoints.size())
    {
      const auto latest_checkpoint =
        state.progress.reached_checkpoints[dep.on_route];

      if (dep.on_checkpoint <= latest_checkpoint)
      {
        shared->reach();
        return subscription;
      }
    }
  }

  _pimpl->dependencies.add(dep, std::move(shared));

  return subscription;
}

//==============================================================================
ProgressVersion Mirror::get_current_progress_version(
  ParticipantId participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return 0;

  return p->second.progress.version;
}

//==============================================================================
std::shared_ptr<const Snapshot> Mirror::snapshot() const
{
  using SnapshotType =
    SnapshotImplementation<
      Implementation::RouteEntry,
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
      state.storage.clear();

      // Insert new routes that are equivalent to the old ones, but which have
      // the updated description.
      for (const auto& [storage_id, route_storage] : old_storage)
      {
        const auto& entry = *route_storage.entry;
        const auto& route = entry.route;
        _pimpl->add_route(
          participant_id, state, entry.route_id, storage_id, route);
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

    if (state.current_plan_id != p.additions().plan_id())
      state.progress = Progress();

    state.current_plan_id = p.additions().plan_id();
    _pimpl->add_routes(participant, state, p.additions());
    state.progress.resize(state.storage.size());

    if (p.progress().has_value())
    {
      state.progress.reached_checkpoints = p.progress()->checkpoints();
      state.progress.version = p.progress()->version();
    }
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

      p_it->second.storage.erase(route.storage_id);
    }
  }

  _pimpl->latest_version = patch.latest_version();

  for (const auto& p : patch)
  {
    _pimpl->dependencies.deprecate_dependencies_before(
      p.participant_id(), p.additions().plan_id());

    if (p.progress().has_value())
    {
      _pimpl->dependencies.reached(
        p.participant_id(), p.additions().plan_id(),
        p.progress()->checkpoints());
    }

    // This is a hacky way of recognizing if a clear() has happened
    const auto& state = _pimpl->states[p.participant_id()];
    if (state.storage.empty())
    {
      _pimpl->dependencies.deprecate_dependencies_before(
        p.participant_id(), p.additions().plan_id()+1);
    }
  }

  return true;
}

//==============================================================================
Database Mirror::fork() const
{
  Database output;

  try
  {
    for (const auto& [id, description] : _pimpl->descriptions)
    {
      ItineraryVersion v = std::numeric_limits<ItineraryVersion>::max();
      const auto p_it = _pimpl->states.find(id);
      if (p_it != _pimpl->states.end())
        v = p_it->second.itinerary_version-1;

      internal_register_participant(output, id, v, *description);
    }

    for (const auto& [participant, state] : _pimpl->states)
    {
      std::vector<RouteStorageInfo> routes;
      routes.reserve(state.storage.size());
      for (const auto& [storage_id, route] : state.storage)
      {
        routes.emplace_back(
          RouteStorageInfo{
            route.entry->route_id,
            storage_id,
            route.entry->route
          });
      }

      std::size_t next_storage_id = 0;
      if (state.highest_storage.has_value())
        next_storage_id = *state.highest_storage + 1;

      set_participant_state(
        output,
        participant,
        state.current_plan_id,
        routes,
        next_storage_id,
        state.itinerary_version,
        state.progress.reached_checkpoints,
        state.progress.version);
    }

    set_initial_fork_version(output, _pimpl->latest_version);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error(
      std::string("[rmf_traffic::schedule::Mirror] ") + e.what());
  }

  return output;
}

} // schedule
} // rmf_traffic
