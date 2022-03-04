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
  _version_mismatch_limiter(1min, 5),
  _assign_plan_id(
    std::make_shared<AssignIDPtr::element_type>(_current_plan_id+1))
{
  // Do nothing
}

//==============================================================================
void Participant::Implementation::Shared::set(
  PlanId plan, std::vector<Route> itinerary)
{
  if (rmf_utils::modular(plan).less_than_or_equal(_current_plan_id))
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[Participant::set] The given plan ID [" + std::to_string(plan)
      + "] needs to be greater than the current ["
      + std::to_string(_current_plan_id) + "]");
    // *INDENT-ON*
  }

  // TODO(MXG): Consider issuing an exception or warning when a single-point
  // trajectory is submitted.
  const auto r_it = std::remove_if(itinerary.begin(), itinerary.end(),
      [](const auto& r) { return r.trajectory().size() < 2; });
  itinerary.erase(r_it, itinerary.end());

  if (itinerary.empty())
  {
    // This situation is more efficient to express as a clear() command
    clear();
    return;
  }

  _change_history.clear();
  _cumulative_delay = std::chrono::seconds(0);
  _current_plan_id = plan;
  _assign_plan_id->fast_forward_to(plan);
  _current_itinerary = std::move(itinerary);

  const ItineraryVersion itinerary_version = get_next_version();
  const ParticipantId id = _id;
  auto change =
    [
      self = weak_from_this(),
      itinerary = _current_itinerary,
      itinerary_version,
      id,
      plan
    ]()
    {
      if (const auto me = self.lock())
        me->_writer->set(id, plan, itinerary, itinerary_version);
    };

  _change_history[itinerary_version] = change;
  change();
}

//==============================================================================
void Participant::Implementation::Shared::clear()
{
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
  const ItineraryVersion last_known_version)
{
  if (rmf_utils::modular(current_version()).less_than(last_known_version))
  {
    if (_version_mismatch_limiter.reached_limit())
    {
      // TODO(MXG): Consider ways for this to be sent to a log instead of just
      // printed to a terminal. Maybe a static object that allows users to
      // set a logger?
      std::cerr
        << "[Participant::Implementation::retransmit] Remote database has a "
        << "higher version number [" << last_known_version << "] than ["
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
      _version = last_known_version+1;
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
  const auto tail_begin = _change_history.upper_bound(last_known_version);
  for (auto it = tail_begin; it != _change_history.end(); ++it)
    it->second();
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
void Participant::set(PlanId plan, std::vector<Route> itinerary)
{
  return _pimpl->_shared->set(plan, std::move(itinerary));
}

//==============================================================================
void Participant::extend(const std::vector<Route>& additional_routes)
{
  const auto& p = _pimpl->_shared;

  p->_current_itinerary.reserve(
    p->_current_itinerary.size() + additional_routes.size());

  for (const auto& item : additional_routes)
    p->_current_itinerary.push_back(item);

  const ItineraryVersion itinerary_version = p->get_next_version();
  const ParticipantId id = p->_id;
  auto change =
    [self = p.get(), additional_routes, itinerary_version, id]()
    {
      // See note in Participant::set::[lambda<change>]
      self->_writer->extend(id, additional_routes, itinerary_version);
    };

  p->_change_history[itinerary_version] = change;
  change();
}

//==============================================================================
void Participant::delay(Duration delay)
{
  const auto& p = _pimpl->_shared;

  bool no_delays = true;
  for (auto& route : p->_current_itinerary)
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

  p->_cumulative_delay += delay;

  const ItineraryVersion itinerary_version = p->get_next_version();
  const ParticipantId id = p->_id;
  auto change =
    [self = p.get(), delay, itinerary_version, id]()
    {
      // See note in Participant::set::[lambda<change>]
      self->_writer->delay(id, delay, itinerary_version);
    };

  p->_change_history[itinerary_version] = change;
  change();
}

//==============================================================================
rmf_traffic::Duration Participant::delay() const
{
  return _pimpl->_shared->_cumulative_delay;
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
PlanId Participant::current_plan_id() const
{
  return _pimpl->_shared->_current_plan_id;
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
