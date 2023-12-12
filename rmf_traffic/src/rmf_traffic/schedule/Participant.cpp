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

#include "internal_Participant.hpp"
#include "debug_Participant.hpp"
#include "internal_Rectifier.hpp"

#include <iostream>
#include <thread>

using namespace std::chrono_literals;

namespace rmf_traffic {
namespace schedule {

//==============================================================================
ItineraryVersion Participant::Debug::get_itinerary_version(const Participant& p)
{
  return p._pimpl->_shared->current_version();
}

//==============================================================================
Participant::Implementation::Shared::Shared(
  const Writer::Registration& registration,
  ParticipantDescription description,
  std::shared_ptr<Writer> writer)
: _id(registration.id()),
  _version(registration.last_itinerary_version()),
  _description(std::move(description)),
  _writer(std::move(writer)),
  _current_plan_id(registration.last_plan_id()),
  _next_storage_base(registration.next_storage_base()),
  _version_mismatch_limiter(1min, 5),
  _assign_plan_id(
    std::make_shared<AssignIDPtr::element_type>(_current_plan_id+1))
{
  // Do nothing
}

//==============================================================================
bool Participant::Implementation::Shared::set(
  PlanId plan, std::vector<Route> itinerary)
{
  if (rmf_utils::modular(plan).less_than_or_equal(_current_plan_id))
    return false;

  for (std::size_t i = 0; i < itinerary.size(); ++i)
  {
    const auto& r = itinerary[i];
    if (r.trajectory().size() < 2)
      return false;
  }

  if (itinerary.empty())
  {
    // This situation is more efficient to express as a clear() command
    clear();
    return true;
  }

  _change_history.clear();
  _cumulative_delay = std::chrono::seconds(0);
  _current_plan_id = plan;
  const auto storage_base = _next_storage_base;
  _next_storage_base += itinerary.size();
  _assign_plan_id->fast_forward_to(plan+1);
  _current_itinerary = std::move(itinerary);
  _progress = _buffered_progress.pull(plan, _current_itinerary.size());

  const ItineraryVersion itinerary_version = get_next_version();
  const ParticipantId id = _id;
  auto change =
    [
      self = weak_from_this(),
      itinerary = _current_itinerary,
      itinerary_version,
      id,
      plan,
      storage_base
    ]()
    {
      if (const auto me = self.lock())
        me->_writer->set(id, plan, itinerary, storage_base, itinerary_version);
    };

  _change_history[itinerary_version] = change;
  change();

  if (_progress.version > 0)
  {
    _writer->reached(
      _id, plan, _progress.reached_checkpoints, _progress.version);
  }

  return true;
}

//==============================================================================
bool Participant::Implementation::Shared::cumulative_delay(
  PlanId plan,
  Duration new_cumulative_delay,
  Duration tolerance)
{
  if (plan != _current_plan_id)
    return false;

  const auto change_in_delay = new_cumulative_delay - _cumulative_delay;
  if (std::chrono::abs(change_in_delay) <= std::chrono::abs(tolerance))
    return true;

  bool no_delays = true;
  for (auto& route : _current_itinerary)
  {
    if (route.trajectory().size() > 0)
    {
      no_delays = false;
      route.trajectory().front().adjust_times(change_in_delay);
    }
  }

  if (no_delays)
  {
    // We don't need to make any changes, because there are no waypoints to move
    return true;
  }

  _cumulative_delay = new_cumulative_delay;
  const ItineraryVersion itinerary_version = get_next_version();
  const ParticipantId id = _id;
  auto change =
    [self = weak_from_this(), change_in_delay, itinerary_version, id]()
    {
      if (const auto me = self.lock())
        me->_writer->delay(id, change_in_delay, itinerary_version);
    };

  _change_history[itinerary_version] = change;
  change();
  return true;
}

//==============================================================================
std::optional<Duration> Participant::Implementation::Shared::cumulative_delay(
  const PlanId plan) const
{
  if (plan == _current_plan_id)
    return _cumulative_delay;

  return std::nullopt;
}

//==============================================================================
void Participant::Implementation::Shared::delay(Duration delay)
{
  bool no_delays = true;
  for (auto& route : _current_itinerary)
  {
    if (route.trajectory().size() > 0)
    {
      no_delays = false;
      route.trajectory().front().adjust_times(delay);
    }
  }

  if (no_delays)
  {
    // We don't need to make any changes, because there are no waypoints to move
    return;
  }

  _cumulative_delay += delay;

  const ItineraryVersion itinerary_version = get_next_version();
  const ParticipantId id = _id;
  auto change =
    [self = weak_from_this(), delay, itinerary_version, id]()
    {
      if (const auto me = self.lock())
        me->_writer->delay(id, delay, itinerary_version);
    };

  _change_history[itinerary_version] = change;
  change();
}

//==============================================================================
void Participant::Implementation::Shared::reached(
  PlanId plan, RouteId route, CheckpointId checkpoint)
{
  if (plan != _current_plan_id)
  {
    if (rmf_utils::modular(plan).less_than(_current_plan_id))
      return;

    _buffered_progress.buff(plan, route, checkpoint);
    return;
  }

  if (_progress.update(route, checkpoint))
  {
    _writer->reached(
      _id, plan, _progress.reached_checkpoints, _progress.version);
  }
}

//==============================================================================
void Participant::Implementation::Shared::clear()
{
  _cumulative_delay = std::chrono::seconds(0);
  if (_current_itinerary.empty())
  {
    // There is nothing to clear, so we can skip this change
    return;
  }

  _current_itinerary.clear();

  const ItineraryVersion itinerary_version = get_next_version();
  const ParticipantId id = _id;
  auto change =
    [self = weak_from_this(), itinerary_version, id]()
    {
      if (const auto me = self.lock())
        me->_writer->clear(id, itinerary_version);
    };

  _change_history[itinerary_version] = change;
  change();
}

//==============================================================================
Participant Participant::Implementation::make(
  ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  const auto registration = writer->register_participant(description);

  Participant participant;
  participant._pimpl = rmf_utils::make_unique_impl<Implementation>(
    registration, std::move(description), std::move(writer));

  if (rectifier_factory)
  {
    participant._pimpl->_shared->_rectification =
      rectifier_factory->make(
      Rectifier::Implementation::make(
        participant._pimpl->_shared), registration.id());
  }

  return participant;
}

//==============================================================================
void Participant::Implementation::Shared::retransmit(
  const std::vector<Rectifier::Range>& ranges,
  const ItineraryVersion last_known_itinerary,
  const ProgressVersion last_known_progress)
{
  if (rmf_utils::modular(current_version()).less_than(last_known_itinerary))
  {
    if (_version_mismatch_limiter.reached_limit())
    {
      // TODO(MXG): Consider ways for this to be sent to a log instead of just
      // printed to a terminal. Maybe a static object that allows users to
      // set a logger?
      std::cerr
        << "[Participant::Implementation::retransmit] Remote database has a "
        << "higher version number [" << last_known_itinerary << "] than ["
        << current_version() << "] the version number of the local "
        << "participant [" << _id << ":" << _description.owner() << "/"
        << _description.name() << "]. This may indicate that the remote "
        << "database is receiving participant updates from a conflicting "
        << "source." << std::endl;
    }
    else
    {
      // Remake the routes to send as a new message with a higher version than
      // the previous
      _version = last_known_itinerary+1;
      set(_current_plan_id, _current_itinerary);
    }

    return;
  }

  for (const auto& range : ranges)
  {
    assert(rmf_utils::modular(range.lower).less_than_or_equal(range.upper));

    // We use lower_bound because it is possible that some of this range has
    // been truncated because of a nullifying change. Therefore we should accept
    // the lowest change in the map if the lower end of this range is gone.
    const auto begin_it = _change_history.lower_bound(range.lower);

    const auto end_it = _change_history.find(range.upper);
    if (end_it == _change_history.end())
    {
      // These changes are no longer relevant. The inconsistency should get
      // fixed by either one of the later ranges or by fixing the tail at the
      // end of this function.
      continue;
    }

    // The _change_map will always contain complete sequences with no gaps, so
    // if end_it was found, then begin_it should be an entry less than or equal
    // to end_it.
    assert(begin_it->first <= end_it->first);

    for (auto it = begin_it; it->first <= end_it->first; ++it)
      it->second();
  }

  // In case the database doesn't have the most recent changes, we will
  // retransmit them.
  const auto tail_begin = _change_history.upper_bound(last_known_itinerary);
  for (auto it = tail_begin; it != _change_history.end(); ++it)
    it->second();

  bool resend_progress = last_known_progress < _progress.version;
  if (!_change_history.empty())
  {
    // If the last known itinerary predates the current history then that
    // itinerary belonged to a different plan, so that is another case where
    // we should resend the current progress.
    resend_progress |= rmf_utils::modular(
      last_known_itinerary).less_than(_change_history.begin()->first);
  }

  if (resend_progress)
  {
    _writer->reached(
      _id, _current_plan_id, _progress.reached_checkpoints, _progress.version);
  }
}

//==============================================================================
ItineraryVersion Participant::Implementation::Shared::current_version() const
{
  return _version;
}

//==============================================================================
ParticipantId Participant::Implementation::Shared::get_id() const
{
  return _id;
}

//==============================================================================
void Participant::Implementation::Shared::change_profile(Profile new_profile)
{
  _description.profile(std::move(new_profile));

  // The register_participant function does not return until it has received a
  // response from the schedule. This will be a problem if a user calls the
  // change_profile function from inside of a callback that inside a spinning
  // executor, because the node will no longer be able to spin and therefore the
  // response will never arrive and therefore the node will no longer be able to
  // spin, creating a deadlock. We run the function in a detached thread so it
  // can do its work without blocking. We don't care about the return value so
  // we can just discard that without syncing.
  std::thread t([w = _writer, d = _description]()
    {
      w->register_participant(d);
    });
  t.detach();
}

//==============================================================================
void Participant::Implementation::Shared::correct_id(ParticipantId new_id)
{
  _id = new_id;

  // Resend the routes because the database certainly has not received them if
  // it disagreed about our participant ID.
  set(_current_plan_id, _current_itinerary);
}

//==============================================================================
const ParticipantDescription&
Participant::Implementation::Shared::get_description() const
{
  return _description;
}

//==============================================================================
Participant::Implementation::Implementation(
  const Writer::Registration& registration,
  ParticipantDescription description,
  std::shared_ptr<Writer> writer)
: _shared(std::make_shared<Shared>(
      registration, std::move(description), std::move(writer)))
{
  // Do nothing
}

//==============================================================================
Participant::Implementation::Shared::~Shared()
{
  // Unregister the participant during destruction
  _writer->unregister_participant(_id);
}

//==============================================================================
ItineraryVersion Participant::Implementation::Shared::get_next_version()
{
  return ++_version;
}

//==============================================================================
bool Participant::set(PlanId plan, std::vector<Route> itinerary)
{
  return _pimpl->_shared->set(plan, std::move(itinerary));
}

//==============================================================================
bool Participant::cumulative_delay(
  PlanId plan, Duration delay, Duration tolerance)
{
  return _pimpl->_shared->cumulative_delay(plan, delay, tolerance);
}

//==============================================================================
std::optional<Duration> Participant::cumulative_delay(PlanId plan) const
{
  return _pimpl->_shared->cumulative_delay(plan);
}

//==============================================================================
void Participant::delay(Duration delay)
{
  return _pimpl->_shared->delay(delay);
}

//==============================================================================
rmf_traffic::Duration Participant::delay() const
{
  return _pimpl->_shared->_cumulative_delay;
}

//==============================================================================
void Participant::reached(PlanId plan, RouteId route, CheckpointId checkpoint)
{
  return _pimpl->_shared->reached(plan, route, checkpoint);
}

//==============================================================================
const std::vector<CheckpointId>& Participant::reached() const
{
  return _pimpl->_shared->_progress.reached_checkpoints;
}

//==============================================================================
void Participant::clear()
{
  _pimpl->_shared->clear();
}

//==============================================================================
const Itinerary& Participant::itinerary() const
{
  return _pimpl->_shared->_current_itinerary;
}

//==============================================================================
ItineraryVersion Participant::version() const
{
  return _pimpl->_shared->_version;
}

//==============================================================================
ProgressVersion Participant::progress_version() const
{
  return _pimpl->_shared->_progress.version;
}

//==============================================================================
const ParticipantDescription& Participant::description() const
{
  return _pimpl->_shared->_description;
}

//==============================================================================
ParticipantId Participant::id() const
{
  return _pimpl->_shared->_id;
}

//==============================================================================
auto Participant::plan_id_assigner() const -> const AssignIDPtr&
{
  return _pimpl->_shared->_assign_plan_id;
}

//==============================================================================
PlanId Participant::assign_plan_id() const
{
  return _pimpl->_shared->_assign_plan_id->assign();
}

//==============================================================================
PlanId Participant::current_plan_id() const
{
  return _pimpl->_shared->_current_plan_id;
}

//==============================================================================
void Participant::change_profile(Profile new_profile)
{
  _pimpl->_shared->change_profile(std::move(new_profile));
}

//==============================================================================
Participant::Participant()
{
  // Do nothing
}

//==============================================================================
Participant make_participant(ParticipantDescription description,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  if (!writer)
  {
    throw std::runtime_error(
            "[rmf_traffic::schedule::make_participant] A nullptr was given for "
            "the `writer` argument. This is illegal.");
  }

  return Participant::Implementation::make(
    std::move(description),
    std::move(writer),
    std::move(rectifier_factory));
}

} // namespace schedule
} // namespace rmf_traffic
