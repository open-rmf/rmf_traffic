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

#include "ChangeInternal.hpp"
#include "InconsistenciesInternal.hpp"
#include "Timeline.hpp"
#include "ViewerInternal.hpp"
#include "debug_Database.hpp"
#include "internal_Snapshot.hpp"
#include "internal_Database.hpp"

#include "../detail/internal_bidirectional_iterator.hpp"
#include "internal_Progress.hpp"
#include "internal_Viewer.hpp"
#include "DependencyTracker.hpp"

#include <rmf_utils/Modular.hpp>

#include <algorithm>
#include <list>
namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Database::Implementation
{
public:

  struct ParticipantState;

  struct Transition;
  using TransitionPtr = std::unique_ptr<Transition>;
  using ConstTransitionPtr = std::unique_ptr<const Transition>;

  struct RouteEntry;
  using RouteEntryPtr = std::shared_ptr<RouteEntry>;

  struct RouteStorage
  {
    RouteEntryPtr entry;
    std::shared_ptr<void> timeline_handle;
  };

  struct Transition
  {
    // If this has a delay value, then the change is a delay to the route.
    // If this contains a nullopt, then the change is an erasure.
    rmf_utils::optional<Change::Delay::Implementation> delay;

    // The previous route entry that this transition is based on.
    RouteStorage predecessor;
  };

  struct RouteEntry : public BaseRouteEntry
  {
    // ===== Additional fields for this timeline entry =====
    // TODO(MXG): Consider defining a base Timeline::Entry class, and then use
    // templates to automatically mix these custom fields with the required
    // fields of the base Entry
    Version schedule_version;
    TransitionPtr transition;
    std::weak_ptr<RouteEntry> successor;

    RouteEntry(
      ConstRoutePtr route_,
      ParticipantId participant_,
      PlanId plan_id_,
      RouteId route_id_,
      StorageId storage_id_,
      std::shared_ptr<const ParticipantDescription> desc_,
      Version schedule_version_,
      TransitionPtr transition_,
      std::weak_ptr<RouteEntry> successor_)
    : BaseRouteEntry{
        std::move(route_),
        participant_,
        plan_id_,
        route_id_,
        storage_id_,
        std::move(desc_),
    },
      schedule_version(schedule_version_),
      transition(std::move(transition_)),
      successor(std::move(successor_))
    {
      // Do nothing
    }
  };

  Timeline<RouteEntry> timeline;

  using ParticipantStorage = std::unordered_map<RouteId, RouteStorage>;

  struct ParticipantState
  {
    std::vector<StorageId> active_routes;
    std::unique_ptr<InconsistencyTracker> tracker;
    ParticipantStorage storage;
    std::shared_ptr<const ParticipantDescription> description;
    const Version initial_schedule_version;
    Version last_updated;
    StorageId next_storage_id = 0;
    PlanId latest_plan_id = std::numeric_limits<RouteId>::max();

    Progress progress = {};
    std::optional<Version> schedule_version_of_progress = std::nullopt;
    ProgressBuffer buffered_progress = {};
  };
  using ParticipantStates = std::unordered_map<ParticipantId, ParticipantState>;
  ParticipantStates states;

  // This violates the single-source-of-truth principle, but it helps make it
  // more efficient to create snapshots
  using ParticipantDescriptions =
    std::unordered_map<ParticipantId,
      std::shared_ptr<const ParticipantDescription>
    >;
  ParticipantDescriptions descriptions;

  using ParticipantRegistrationVersions = std::map<Version, ParticipantId>;
  ParticipantRegistrationVersions add_participant_version;

  struct RemoveParticipantInfo
  {
    ParticipantId id;
    Version original_version;
  };
  using ParticipantUnregistrationVersion =
    std::map<Version, RemoveParticipantInfo>;
  ParticipantUnregistrationVersion remove_participant_version;

  using ParticipantRegistrationTime = std::map<Time, Version>;
  ParticipantRegistrationTime remove_participant_time;

  // NOTE(MXG): We store this record of inconsistency ranges here as a single
  // group that covers all participants in order to make it easy for us to share
  // it externally using the inconsistencies() function.
  //
  // This field should only be modified by InconsistencyTracker or
  // unregister_participant. We are also trusting the InconsistencyTracker
  // instance of each ParticipantState to not touch any other ParticipantState's
  // entry in this field.
  Inconsistencies inconsistencies;

  using StagedChanges =
    std::unordered_map<
    ParticipantId,
    std::map<ItineraryVersion, Change>>;
  StagedChanges staged_changes;

  std::unordered_set<ParticipantId> participant_ids;

  Version schedule_version = 0;

  /// When the Database forked off of a Mirror, this struct will contain
  /// information about how the mirror was initialized
  struct ForkInitializationInfo
  {
    Version initial_version;
    Time initial_version_maximum_time;
  };
  std::optional<ForkInitializationInfo> fork_info;

  struct CullInfo
  {
    Change::Cull cull;
    Version version;
  };

  std::optional<CullInfo> last_cull;

  /// The current time is used to know when participants can be culled after
  /// getting unregistered
  rmf_traffic::Time current_time = rmf_traffic::Time(rmf_traffic::Duration(0));

  mutable DependencyTracker dependencies;

  /// This function is used to insert routes into the Database.
  void insert_items(
    const ParticipantId participant,
    ParticipantState& state,
    const Itinerary& itinerary)
  {
    ParticipantStorage& storage = state.storage;

    const auto plan_id = state.latest_plan_id;
    const auto initial_route_num = state.active_routes.size();
    for (std::size_t i = 0; i < itinerary.size(); ++i)
    {
      const auto& route = itinerary[i];
      const auto route_id = i + initial_route_num;
      const auto storage_id = state.next_storage_id++;

      state.active_routes.push_back(storage_id);

      RouteStorage& entry_storage = storage[storage_id];
      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          std::make_shared<Route>(route),
          participant,
          plan_id,
          route_id,
          storage_id,
          state.description,
          schedule_version,
          nullptr,
          RouteEntryPtr()
        });

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }

  void apply_delay(
    ParticipantId participant,
    ParticipantState& state,
    Duration delay)
  {
    ParticipantStorage& storage = state.storage;
    for (const StorageId storage_id : state.active_routes)
    {
      assert(storage.find(storage_id) != storage.end());
      auto& entry_storage = storage.at(storage_id);
      const auto& route_entry = entry_storage.entry;
      const auto route_id = route_entry->route_id;

      const Trajectory& old_trajectory = route_entry->route->trajectory();
      assert(old_trajectory.start_time());
      if (old_trajectory.empty())
        continue;

      auto new_route = std::make_shared<Route>(*route_entry->route);
      new_route->trajectory().front().adjust_times(delay);

      auto transition = std::make_unique<Transition>(
        Transition{
          Change::Delay::Implementation{delay},
          std::move(entry_storage)
        });

      // NOTE(MXG): The previous contents of entry have been moved into the
      // predecessor field of transition, so we are free to refill entry with
      // the newly created data.
      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          std::move(new_route),
          participant,
          state.latest_plan_id,
          route_id,
          storage_id,
          state.description,
          schedule_version,
          std::move(transition),
          RouteEntryPtr()
        });

      entry_storage.entry->transition->predecessor.entry->successor =
        entry_storage.entry;

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }

  void apply_description_update(
    ParticipantId participant,
    ParticipantState& state)
  {
    ParticipantStorage& storage = state.storage;
    for (const StorageId storage_id : state.active_routes)
    {
      assert(storage.find(storage_id) != storage.end());
      auto& entry_storage = storage.at(storage_id);

      auto route = entry_storage.entry->route;
      const auto route_id = entry_storage.entry->route_id;

      auto transition = std::make_unique<Transition>(
        Transition{
          std::nullopt,
          std::move(entry_storage)
        });

      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          std::move(route),
          participant,
          state.latest_plan_id,
          route_id,
          storage_id,
          state.description,
          schedule_version,
          std::move(transition),
          RouteEntryPtr()
        });

      entry_storage.entry->transition->predecessor.entry->successor =
        entry_storage.entry;

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }
  }

  void clear(
    ParticipantId participant,
    ParticipantState& state)
  {
    ParticipantStorage& storage = state.storage;
    for (const StorageId storage_id : state.active_routes)
    {
      assert(storage.find(storage_id) != storage.end());
      auto& entry_storage = storage.at(storage_id);
      const auto& entry = *entry_storage.entry;

      auto transition = std::make_unique<Transition>(
        Transition{
          rmf_utils::nullopt,
          std::move(entry_storage)
        });

      entry_storage.entry = std::make_unique<RouteEntry>(
        RouteEntry{
          nullptr,
          participant,
          entry.plan_id,
          entry.route_id,
          storage_id,
          state.description,
          schedule_version,
          std::move(transition),
          RouteEntryPtr()
        });

      entry_storage.entry->transition->predecessor.entry->successor =
        entry_storage.entry;

      entry_storage.timeline_handle = timeline.insert(entry_storage.entry);
    }

    state.active_routes.clear();
    state.progress.reached_checkpoints.clear();
  }

  ParticipantId get_next_participant_id()
  {
    // This will cycle through the set of currently active participant IDs until
    // it finds a value which is not already taken. If it cycles through the
    // entire set of possible values and cannot find a value that is available,
    // then we will quit and throw an exception. Note that it is
    // incomprehensible to have that many participants in the schedule.
    const ParticipantId initial_suggestion = _next_participant_id;
    do
    {
      const auto insertion = participant_ids.insert(_next_participant_id);
      ++_next_participant_id;
      if (insertion.second)
        return *insertion.first;

    } while (_next_participant_id != initial_suggestion);

    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::Implementation::get_next_participant_id] There are no "
      "remaining Participant ID values available. This should never happen."
      " Please report this as a serious bug.");
    // *INDENT-ON*
  }

  void add_new_participant_id(ParticipantId new_id)
  {
    if (rmf_utils::modular(_next_participant_id).less_than_or_equal(new_id))
      _next_participant_id = new_id + 1;

    const auto insertion = participant_ids.insert(new_id);
    if (!insertion.second)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[Database::Implementation::add_new_participant_id] Re-adding "
        "participant ID [" + std::to_string(new_id) + "]. This should not be "
        "possible! Please report this bug.");
      // *INDENT-ON*
    }
  }

  static Implementation& get(Database& database)
  {
    return *database._pimpl;
  }

private:
  ParticipantId _next_participant_id = 0;
};

//==============================================================================
std::size_t Database::Debug::current_entry_history_count(
  const Database& database)
{
  std::size_t count = 0;
  for (const auto& p : database._pimpl->states)
    count += p.second.storage.size();

  return count;
}

//==============================================================================
std::size_t Database::Debug::current_removed_participant_count(
  const Database& database)
{
  return database._pimpl->remove_participant_version.size();
}

//==============================================================================
std::optional<Itinerary> Database::Debug::get_itinerary(
  const Database& database,
  const ParticipantId participant)
{
  const auto state_it = database._pimpl->states.find(participant);
  if (state_it == database._pimpl->states.end())
    return rmf_utils::nullopt;

  const Implementation::ParticipantState& state = state_it->second;

  Itinerary itinerary;
  itinerary.reserve(state.active_routes.size());
  for (const RouteId route : state.active_routes)
    itinerary.push_back(*state.storage.at(route).entry->route);

  return itinerary;
}

//==============================================================================
void Database::set(
  const ParticipantId participant,
  const PlanId plan,
  const Itinerary& itinerary,
  const StorageId storage_base,
  const ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_traffic::schedule::Database::set] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version, true))
  {
    ticket->set([=]() {
      this->set(participant, plan, itinerary, storage_base, version);
    });
    return;
  }

  ++_pimpl->schedule_version;

  // Erase the routes that are currently active
  _pimpl->clear(participant, state);

  // Reset the current progress of the state
  state.progress = state.buffered_progress.pull(plan, itinerary.size());
  state.schedule_version_of_progress = _pimpl->schedule_version;

  // Insert the new routes into the current itinerary
  state.latest_plan_id = plan;
  state.next_storage_id = storage_base;
  _pimpl->insert_items(participant, state, itinerary);

  // Deprecate all dependencies on earlier plans
  _pimpl->dependencies.deprecate_dependencies_before(
        participant, plan);
  _pimpl->dependencies.reached(
    participant, plan, state.progress.reached_checkpoints);
}

//==============================================================================
void Database::extend(
  ParticipantId participant,
  const Itinerary& itinerary,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[rmf_traffic::schedule::Database::extend] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  // Check if the version on this change has any inconsistencies
  if (auto ticket = state.tracker->check(version))
  {
    // If we got a ticket from the inconsistency tracker, then pass along a
    // callback to call this
    ticket->set([=]() { this->extend(participant, itinerary, version); });
    return;
  }

  ++_pimpl->schedule_version;

  _pimpl->insert_items(participant, state, itinerary);

  // Update the progress tracker with the new routes
  state.progress.resize(state.active_routes.size());
  state.schedule_version_of_progress = _pimpl->schedule_version;
}

//==============================================================================
void Database::delay(
  ParticipantId participant,
  Duration delay,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::delay] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=]() { this->delay(participant, delay, version); });
    return;
  }

  ++_pimpl->schedule_version;
  _pimpl->apply_delay(participant, state, delay);
}

//==============================================================================
void Database::reached(
  ParticipantId participant,
  PlanId plan,
  const std::vector<CheckpointId>& reached_checkpoints,
  ProgressVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::reached] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }
  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (plan != state.latest_plan_id)
  {
    if (rmf_utils::modular(plan).less_than(state.latest_plan_id))
      return;

    for (std::size_t i = 0; i < reached_checkpoints.size(); ++i)
      state.buffered_progress.buff(plan, i, reached_checkpoints[i], version);

    return;
  }

  for (std::size_t i = 0; i < reached_checkpoints.size(); ++i)
    state.progress.update(i, reached_checkpoints[i], version);

  state.schedule_version_of_progress = ++_pimpl->schedule_version;

  // Update relevant dependencies
  _pimpl->dependencies.reached(
    participant, plan, state.progress.reached_checkpoints);
}

//==============================================================================
void Database::clear(
  ParticipantId participant,
  ItineraryVersion version)
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::erase] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  Implementation::ParticipantState& state = p_it->second;

  assert(state.tracker);
  if (rmf_utils::modular(version).less_than(state.tracker->expected_version()))
  {
    // This is an old change, possibly a retransmission requested by a different
    // database tracker, so we will ignore it.
    return;
  }

  if (auto ticket = state.tracker->check(version))
  {
    ticket->set([=]() { this->clear(participant, version); });
    return;
  }

  ++_pimpl->schedule_version;
  _pimpl->clear(participant, state);
  _pimpl->dependencies.deprecate_dependencies_before(
    participant, state.latest_plan_id+1);
}

//==============================================================================
Writer::Registration register_participant_impl(
  Database::Implementation& pimpl,
  ParticipantId id,
  ItineraryVersion last_known_version,
  ParticipantDescription description)
{
  const Version version = ++pimpl.schedule_version;
  auto tracker = Inconsistencies::Implementation::register_participant(
    pimpl.inconsistencies, id, last_known_version);

  const auto description_ptr =
    std::make_shared<const ParticipantDescription>(std::move(description));

  const auto p_it = pimpl.states.insert(
    std::make_pair(
      id,
      Database::Implementation::ParticipantState{
        {},
        std::move(tracker),
        {},
        description_ptr,
        version,
        version
      })).first;

  pimpl.descriptions.insert({id, description_ptr});

  pimpl.add_participant_version[version] = id;

  const auto& state = p_it->second;
  return Database::Registration(
    id, state.tracker->last_known_version(),
    state.latest_plan_id, state.next_storage_id);
}

//==============================================================================
Writer::Registration Database::register_participant(
  ParticipantDescription description)
{
  const ParticipantId id = _pimpl->get_next_participant_id();
  return register_participant_impl(
    *_pimpl,
    id,
    std::numeric_limits<ItineraryVersion>::max(),
    std::move(description));
}

//==============================================================================
void internal_register_participant(
  Database& database,
  ParticipantId id,
  ItineraryVersion last_known_version,
  ParticipantDescription description)
{
  auto& impl = Database::Implementation::get(database);
  impl.add_new_participant_id(id);
  register_participant_impl(
    impl, id, last_known_version, std::move(description));
}

//==============================================================================
void set_participant_state(
  Database& database,
  ParticipantId participant,
  PlanId plan,
  std::vector<RouteStorageInfo> routes,
  StorageId storage_base,
  ItineraryVersion itinerary_version,
  std::vector<CheckpointId> progress,
  ProgressVersion progress_version)
{
  using RouteEntry = Database::Implementation::RouteEntry;
  using RouteEntryPtr = Database::Implementation::RouteEntryPtr;
  auto& impl = Database::Implementation::get(database);
  const auto p_it = impl.states.find(participant);
  if (p_it == impl.states.end())
  {
    // *INDENT-OFF*
    // The [rmf_traffic::schedule::Mirror] tag will be added to this message
    // when it gets caught by that function.
    throw std::runtime_error(
      "No participant with ID [" + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  auto& state = p_it->second;

  // This should only be used by Mirror::fork, so the given value should always
  // perfectly match the expectation.
  assert(state.tracker->expected_version() == itinerary_version);
  if (auto ticket = state.tracker->check(itinerary_version, true))
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "Inconsistency detected with the itinerary version ["
      + std::to_string(itinerary_version) + "] of participant ["
      + std::to_string(participant));
    // *INDENT-ON*
  }

  state.active_routes.clear();
  state.latest_plan_id = plan;
  state.next_storage_id = storage_base;
  state.progress.reached_checkpoints = std::move(progress);
  state.progress.version = progress_version;
  auto& storage = state.storage;

  for (std::size_t i = 0; i < routes.size(); ++i)
  {
    const auto& info = routes[i];
    const auto storage_id = info.storage_id;

    state.active_routes.push_back(storage_id);

    auto& entry_storage = storage[storage_id];
    entry_storage.entry = std::make_unique<RouteEntry>(
      RouteEntry{
        info.route,
        participant,
        plan,
        i,
        storage_id,
        state.description,
        impl.schedule_version,
        nullptr,
        RouteEntryPtr()
      });

    entry_storage.timeline_handle = impl.timeline.insert(entry_storage.entry);
  }

  std::sort(state.active_routes.begin(), state.active_routes.end());
}

//==============================================================================
void set_initial_fork_version(
  Database& database,
  Version version)
{
  auto& impl = Database::Implementation::get(database);
  impl.schedule_version = version;

  std::optional<Time> maximum_time;
  for (auto& [_, state] : impl.states)
  {
    for (const auto& [_, storage] : state.storage)
    {
      const auto& route = storage.entry->route;
      const auto* finish = route->trajectory().finish_time();
      if (finish)
      {
        if (!maximum_time.has_value() || *maximum_time < *finish)
          maximum_time = *finish;
      }
    }

    state.schedule_version_of_progress = version;
  }

  if (maximum_time.has_value())
  {
    impl.fork_info = Database::Implementation::ForkInitializationInfo{
      version,
      *maximum_time
    };
  }
}

//==============================================================================
void Database::update_description(
  ParticipantId id,
  ParticipantDescription desc)
{
  const auto p_it = _pimpl->states.find(id);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::update_description] No participant with ID ["
      + std::to_string(id) + "]");
    // *INDENT-ON*
  }

  const auto description_ptr =
    std::make_shared<ParticipantDescription>(std::move(desc));

  auto version = ++_pimpl->schedule_version;
  p_it->second.last_updated = version;
  p_it->second.description = description_ptr;
  _pimpl->descriptions[id] = description_ptr;
  _pimpl->apply_description_update(id, p_it->second);
}

//==============================================================================
void Database::unregister_participant(
  ParticipantId participant)
{
  const auto id_it = _pimpl->participant_ids.find(participant);
  const auto state_it = _pimpl->states.find(participant);

  if (id_it == _pimpl->participant_ids.end()
    && state_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::unregister_participant] Requested unregistering an "
      "inactive participant ID [" + std::to_string(participant) + "]");
    // *INDENT-ON*
  }
  else if (id_it == _pimpl->participant_ids.end()
    || state_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::unregister_participant] Inconsistency in participant "
      "registration ["
      + std::to_string(id_it == _pimpl->participant_ids.end()) + ":"
      + std::to_string(state_it == _pimpl->states.end())
      + "]. Please report this as a serious bug!");
    // *INDENT-ON*
  }

  _pimpl->inconsistencies._pimpl->unregister_participant(participant);

  const Version initial_version = state_it->second.initial_schedule_version;
  _pimpl->add_participant_version.erase(initial_version);

  _pimpl->participant_ids.erase(id_it);
  _pimpl->states.erase(state_it);
  _pimpl->descriptions.erase(participant);

  const Version version = ++_pimpl->schedule_version;
  _pimpl->remove_participant_version[version] = {participant, initial_version};
  _pimpl->remove_participant_time[_pimpl->current_time] = version;

  // Deprecate all dependencies for this participant
  _pimpl->dependencies.deprecate_dependencies_on(participant);
}

//==============================================================================
namespace {

//==============================================================================
const Database::Implementation::RouteEntry* get_most_recent(
  const Database::Implementation::RouteEntry* from)
{
  assert(from);
  while (const auto successor = from->successor.lock())
    from = successor.get();

  return from;
}

//==============================================================================
struct Delay
{
  Duration duration;
};

//==============================================================================
struct ParticipantChanges
{
  std::vector<Change::Add::Item> additions;
  std::map<Version, Delay> delays;
  std::vector<RouteId> erasures;
};

//==============================================================================
class PatchRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  PatchRelevanceInspector(Version after)
  : _after(after)
  {
    // Do nothing
  }

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  const RouteEntry* get_last_known_ancestor(const RouteEntry* from) const
  {
    assert(from);
    while (from && rmf_utils::modular(_after).less_than(from->schedule_version))
    {
      if (from->transition)
        from = from->transition->predecessor.entry.get();
      else
        return nullptr;
    }

    while (const auto successor = from->successor.lock())
    {
      if (rmf_utils::modular(_after).less_than(successor->schedule_version))
        break;

      from = successor.get();
    }

    return from;
  }

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    const RouteEntry* const last = get_last_known_ancestor(entry);
    const RouteEntry* const newest = get_most_recent(entry);

    if (last == newest)
    {
      // There are no changes for this route to give the mirror
      return;
    }

    if (last && last->route && relevant(*last))
    {
      // The mirror knew about a previous version of this route
      if (newest->route && relevant(*newest))
      {
        // The newest version of this route is relevant to the mirror
        const RouteEntry* traverse = newest;
        ParticipantChanges& p_changes = changes[newest->participant];
        while (traverse != last)
        {
          const auto* const transition = traverse->transition.get();
          assert(transition);

          if (transition->delay.has_value())
          {
            const auto& delay = *transition->delay;
            const auto insertion = p_changes.delays.insert(
              std::make_pair(
                traverse->schedule_version,
                Delay{
                  delay.duration
                }));
#ifndef NDEBUG
            // When compiling in debug mode, if we see a duplicate insertion,
            // let's make sure that the previously entered data matches what we
            // wanted to enter just now.
            if (!insertion.second)
            {
              const Delay& previous = insertion.first->second;
              assert(previous.duration == delay.duration);
            }
#else
            // When compiling in release mode, cast the return value to void to
            // suppress compiler warnings.
            (void)(insertion);
#endif // NDEBUG
          }

          traverse = traverse->transition->predecessor.entry.get();
        }
      }
      else
      {
        // The newest version of this route is not relevant to the mirror, so
        // we will erase it from the mirror.
        changes[newest->participant].erasures.emplace_back(newest->storage_id);
      }
    }
    else
    {
      // No version of this route has been seen by the mirror
      if (newest->route && relevant(*newest))
      {
        // The newest version of this route is relevant to the mirror
        changes[newest->participant].additions.emplace_back(
          Change::Add::Item{
            newest->route_id,
            newest->storage_id,
            newest->route
          });
      }
      else
      {
        // Ignore this route. The mirror has no need to know about it.
      }
    }
  }

private:
  const Version _after;
};

//==============================================================================
class FirstPatchRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;

  std::unordered_map<ParticipantId, ParticipantChanges> changes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    const RouteEntry* const newest = get_most_recent(entry);
    if (newest->route && relevant(*newest))
    {
      changes[newest->participant].additions.emplace_back(
        Change::Add::Item{
          newest->route_id,
          newest->storage_id,
          newest->route
        });
    }
  }
};

//==============================================================================
class ViewRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    entry = get_most_recent(entry);
    if (entry->route && relevant(*entry))
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
// TODO(MXG): This class is redundant with MirrorViewRelevanceInspector
class SnapshotViewRelevanceInspector
  : public TimelineInspector<BaseRouteEntry>
{
public:

  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  void inspect(
    const BaseRouteEntry* entry,
    const std::function<bool(const BaseRouteEntry&)>& relevant) final
  {
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
class ViewerAfterRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  using RouteEntry = Database::Implementation::RouteEntry;
  using Storage = Viewer::View::Implementation::Storage;

  std::vector<Storage> routes;

  const Version after;

  ViewerAfterRelevanceInspector(Version _after)
  : after(_after)
  {
    // Do nothing
  }

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& relevant) final
  {
    entry = get_most_recent(entry);
    if (rmf_utils::modular(after).less_than(entry->schedule_version)
      && entry->route && relevant(*entry))
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
class CullRelevanceInspector
  : public TimelineInspector<Database::Implementation::RouteEntry>
{
public:

  CullRelevanceInspector(Time cull_time)
  : _cull_time(cull_time)
  {
    // Do nothing
  }

  using RouteEntry = Database::Implementation::RouteEntry;

  struct Info
  {
    ParticipantId participant;
    StorageId storage_id;
  };

  std::vector<Info> routes;

  void inspect(
    const RouteEntry* entry,
    const std::function<bool(const RouteEntry&)>& /*relevant*/) final
  {
    while (const auto successor = entry->successor.lock())
    {
      if (!successor->route)
        break;

      entry = successor.get();
    }

    assert(entry->route->trajectory().finish_time());
    if (*entry->route->trajectory().finish_time() < _cull_time)
    {
      routes.emplace_back(Info{entry->participant, entry->storage_id});
    }
  }

private:
  Time _cull_time;
};

} // anonymous namespace

//==============================================================================
Viewer::View Database::query(const Query& parameters) const
{
  return query(parameters.spacetime(), parameters.participants());
}

//==============================================================================
Viewer::View Database::query(
  const Query::Spacetime& spacetime,
  const Query::Participants& participants) const
{
  ViewRelevanceInspector inspector;
  _pimpl->timeline.inspect(spacetime, participants, inspector);
  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
const std::unordered_set<ParticipantId>& Database::participant_ids() const
{
  return _pimpl->participant_ids;
}

//==============================================================================
std::shared_ptr<const ParticipantDescription> Database::get_participant(
  std::size_t participant_id) const
{
  const auto state_it = _pimpl->descriptions.find(participant_id);
  if (state_it == _pimpl->descriptions.end())
    return nullptr;

  return state_it->second;
}

//==============================================================================
Version Database::latest_version() const
{
  return _pimpl->schedule_version;
}

//==============================================================================
std::optional<ItineraryView> Database::get_itinerary(
  const std::size_t participant_id) const
{
  const auto state_it = _pimpl->states.find(participant_id);
  if (state_it == _pimpl->states.end())
    return std::nullopt;

  const Implementation::ParticipantState& state = state_it->second;

  ItineraryView itinerary;
  itinerary.reserve(state.active_routes.size());
  for (const RouteId route : state.active_routes)
    itinerary.push_back(state.storage.at(route).entry->route);

  return itinerary;
}

//==============================================================================
std::optional<PlanId> Database::get_current_plan_id(
  const std::size_t participant_id) const
{
  const auto state_it = _pimpl->states.find(participant_id);
  if (state_it == _pimpl->states.end())
    return std::nullopt;

  const auto& state = state_it->second;
  return state.latest_plan_id;
}

//==============================================================================
const std::vector<CheckpointId>* Database::get_current_progress(
  ParticipantId participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return nullptr;

  return &p->second.progress.reached_checkpoints;
}

//==============================================================================
ProgressVersion Database::get_current_progress_version(
  ParticipantId participant_id) const
{
  const auto p = _pimpl->states.find(participant_id);
  if (p == _pimpl->states.end())
    return 0;

  return p->second.progress.version;
}

//==============================================================================
auto Database::watch_dependency(
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
  if (rmf_utils::modular(dep.on_plan).less_than(state.latest_plan_id))
  {
    shared->deprecate();
    return subscription;
  }

  if (state.latest_plan_id == dep.on_plan)
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
std::shared_ptr<const Snapshot> Database::snapshot() const
{
  using SnapshotType =
    SnapshotImplementation<BaseRouteEntry, SnapshotViewRelevanceInspector>;

  const auto check_relevant = [](const Implementation::RouteEntry& entry)
    {
      // If this entry has no successor, then it is relevant to the snapshot.
      return entry.successor.lock() == nullptr;
    };

  return std::make_shared<SnapshotType>(
    _pimpl->timeline.snapshot(check_relevant),
    _pimpl->participant_ids,
    _pimpl->descriptions,
    _pimpl->schedule_version);
}

//==============================================================================
Database::Database()
: _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
const Inconsistencies& Database::inconsistencies() const
{
  return _pimpl->inconsistencies;
}

//==============================================================================
auto Database::changes(
  const Query& parameters,
  std::optional<Version> after) const -> Patch
{
  if (after.has_value() && _pimpl->fork_info.has_value())
  {
    // If a specific version is being asked for, but that version predates the
    // initial version of this forked database, then we will instead simply send
    // a changeset that will bring the mirrors up to date no matter what their
    // current version is.
    if (*after < _pimpl->fork_info->initial_version)
      return changes(parameters, std::nullopt);
  }

  std::unordered_map<ParticipantId, ParticipantChanges> changes;
  if (after.has_value())
  {
    PatchRelevanceInspector inspector(*after);
    _pimpl->timeline.inspect(
      parameters.spacetime(), parameters.participants(), inspector);

    changes = inspector.changes;
  }
  else
  {
    FirstPatchRelevanceInspector inspector;
    _pimpl->timeline.inspect(
      parameters.spacetime(), parameters.participants(), inspector);

    changes = inspector.changes;
  }

  std::vector<Patch::Participant> part_patches;
  for (const auto& p : changes)
  {
    const auto& changeset = p.second;
    const auto& state = _pimpl->states.at(p.first);

    std::vector<Change::Delay> delays;
    for (const auto& d : changeset.delays)
    {
      delays.emplace_back(
        Change::Delay{
          d.second.duration
        });
    }

    if (changeset.erasures.empty()
      && delays.empty()
      && changeset.additions.empty())
    {
      // There aren't actually any changes for this participant, so we will
      // leave it out of the patch.
      continue;
    }

    std::optional<Change::Progress> progress;
    if (state.schedule_version_of_progress.has_value())
    {
      if (!after.has_value() || *after < *state.schedule_version_of_progress)
      {
        progress = Change::Progress(
            state.progress.version,
            state.progress.reached_checkpoints);
      }
    }

    part_patches.emplace_back(
      Patch::Participant{
        p.first,
        state.tracker->last_known_version(),
        Change::Erase(std::move(p.second.erasures)),
        std::move(delays),
        Change::Add(state.latest_plan_id, std::move(p.second.additions)),
        std::move(progress)
      });
  }

  std::optional<Change::Cull> cull;
  if (_pimpl->last_cull && after && *after < _pimpl->last_cull->version)
  {
    cull = _pimpl->last_cull->cull;
  }

  return Patch(
    std::move(part_patches),
    cull,
    after,
    _pimpl->schedule_version);
}

//==============================================================================
Viewer::View Database::query(const Query& parameters, const Version after) const
{
  ViewerAfterRelevanceInspector inspector{after};
  _pimpl->timeline.inspect(
    parameters.spacetime(), parameters.participants(), inspector);

  return Viewer::View::Implementation::make_view(std::move(inspector.routes));
}

//==============================================================================
Version Database::cull(Time time)
{
  Query::Spacetime spacetime;
  spacetime.query_timespan().set_upper_time_bound(time);

  CullRelevanceInspector inspector(time);
  _pimpl->timeline.inspect(
    spacetime, Query::Participants::make_all(), inspector);

  // TODO(MXG) This iterating could probably be made more efficient by grouping
  // together the culls of each participant.
  for (const auto& route : inspector.routes)
  {
    auto p_it = _pimpl->states.find(route.participant);
    assert(p_it != _pimpl->states.end());

    auto& storage = p_it->second.storage;
    const auto r_it = storage.find(route.storage_id);
    assert(r_it != storage.end());

    storage.erase(r_it);
  }

  _pimpl->timeline.cull(time);

  // Erase all trace of participants that were removed before the culling time.
  const auto p_cull_begin = _pimpl->remove_participant_time.begin();
  const auto p_cull_end = _pimpl->remove_participant_time.upper_bound(time);
  for (auto p_cull_it = p_cull_begin; p_cull_it != p_cull_end; ++p_cull_it)
  {
    const auto remove_it =
      _pimpl->remove_participant_version.find(p_cull_it->second);
    assert(remove_it != _pimpl->remove_participant_version.end());

    _pimpl->remove_participant_version.erase(remove_it);
  }

  if (p_cull_begin != p_cull_end)
    _pimpl->remove_participant_time.erase(p_cull_begin, p_cull_end);

  // Update the version of the schedule
  ++_pimpl->schedule_version;

  // Record the occurrence of this cull
  _pimpl->last_cull = Implementation::CullInfo{
    Change::Cull(time),
    _pimpl->schedule_version
  };

  if (_pimpl->fork_info.has_value())
  {
    // If the initial state of the fork is being culled, then we can erase this
    // fork initialization information.
    if (_pimpl->fork_info->initial_version_maximum_time < time)
      _pimpl->fork_info = std::nullopt;
  }

  return _pimpl->schedule_version;
}

//==============================================================================
void Database::set_current_time(Time time)
{
  _pimpl->current_time = time;
}

//==============================================================================
ItineraryVersion Database::itinerary_version(ParticipantId participant) const
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Database::itinerary_version] No participant with ID ["
      + std::to_string(participant) + "]");
    // *INDENT-ON*
  }

  return p_it->second.tracker->last_known_version();
}

//==============================================================================
PlanId Database::latest_plan_id(ParticipantId participant) const
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    throw std::runtime_error(
            "[Database::lastest_plan_id] No participant with ID ["
            + std::to_string(participant) + "]");
  }

  return p_it->second.latest_plan_id;
}

//==============================================================================
StorageId Database::next_storage_base(ParticipantId participant) const
{
  const auto p_it = _pimpl->states.find(participant);
  if (p_it == _pimpl->states.end())
  {
    throw std::runtime_error(
            "[Database::latest_storage_id] No participant with ID ["
            + std::to_string(participant) + "]");
  }

  return p_it->second.next_storage_id;
}

} // namespace schedule
} // namespace rmf_traffic
