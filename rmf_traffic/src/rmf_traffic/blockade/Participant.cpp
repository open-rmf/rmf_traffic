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
#include "internal_Rectifier.hpp"

#include <sstream>


#include <iostream>

namespace rmf_traffic {
namespace blockade {

//==============================================================================
Participant Participant::Implementation::make(
  const ParticipantId participant_id,
  const double radius,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  Participant participant;
  participant._pimpl = rmf_utils::make_unique_impl<Implementation>(
    participant_id, radius, std::move(writer));

  if (rectifier_factory)
  {
    participant._pimpl->_shared->_rectification =
      rectifier_factory->make(
      Rectifier::Implementation::make(
        participant._pimpl->_shared), participant_id);
  }

  return participant;
}

//==============================================================================
Participant::Implementation::Shared::Shared(
  const ParticipantId id,
  const double radius,
  std::shared_ptr<Writer> writer)
: _id(id),
  _writer(std::move(writer)),
  _reservation_id(std::nullopt),
  _last_reached(0)
{
  _current_reservation.radius = radius;
}

//==============================================================================
void Participant::Implementation::Shared::check(const Status& status)
{
  if (status.reservation != _reservation_id)
  {
    if (!_reservation_id.has_value())
    {
      // If _reservation_id is a nullopt, but the blockade moderator believes we
      // have a reservation, then we should remind the blockade moderator that
      // we have canceled that reservation.
      _writer->cancel(_id, status.reservation);
      return;
    }

    _send_reservation();

    if (_last_ready)
      _send_ready();

    // We intentionally leave out _last_reached for now, because a blockade
    // participant is expected to sit at the first waypoint until its
    // reservation is recognized by the moderator and departure is granted.
    // Therefore, the participant should not have reached anything past 0 if the
    // moderator does not yet know about the new path.

    return;
  }

  if (status.last_ready != _last_ready)
  {
    if (status.last_ready.has_value() && !_last_ready.has_value())
    {
      // The moderator thinks we are ready for some checkpoints, but we are not
      // ready for any. Releasing checkpoint 0 will tell the moderator that we
      // are not ready for any of these.
      _send_release(0);
    }
    else
    {
      assert(_last_ready.has_value());
      if (status.last_ready.has_value() && *status.last_ready > *_last_ready)
      {
        _send_release(_last_ready.value() + 1);
      }
      else
      {
        // We will reach this condition if:
        // 1. The moderator incorrectly thinks we are not ready for any
        //    departures, or
        // 2. The moderator does not know about our latest ready departure
        _send_ready();
      }
    }
  }

  if (status.last_reached != _last_reached)
    _send_reached();
}

//==============================================================================
void Participant::Implementation::Shared::check()
{
  if (!_reservation_id)
    return;

  _send_reservation();
}

//==============================================================================
Participant::Implementation::Implementation(
  const ParticipantId id,
  const double radius,
  std::shared_ptr<Writer> writer)
: _shared(std::make_shared<Shared>(id, radius, std::move(writer)))
{
  // Do nothing
}

//==============================================================================
Participant::Implementation::Shared::~Shared()
{
  if (_reservation_id)
    _writer->cancel(_id);
}

//==============================================================================
void Participant::Implementation::Shared::_send_reservation()
{
  assert(_current_reservation.path.size() > 1);
  _writer->set(_id, _reservation_id.value(), _current_reservation);
}

//==============================================================================
void Participant::Implementation::Shared::_send_ready()
{
  _writer->ready(_id, _reservation_id.value(), _last_ready.value());
}

//==============================================================================
void Participant::Implementation::Shared::_send_release(CheckpointId checkpoint)
{
  _writer->release(_id, _reservation_id.value(), checkpoint);
}

//==============================================================================
void Participant::Implementation::Shared::_send_reached()
{
  _writer->reached(_id, _reservation_id.value(), _last_reached);
}

//==============================================================================
void Participant::radius(const double new_radius)
{
  _pimpl->_shared->_current_reservation.radius = new_radius;
}

//==============================================================================
double Participant::radius() const
{
  return _pimpl->_shared->_current_reservation.radius;
}

//==============================================================================
void Participant::set(std::vector<Writer::Checkpoint> path)
{
  const auto& p = _pimpl->_shared;
  p->_current_reservation.path = std::move(path);

  if (p->_reservation_id)
    ++*p->_reservation_id;
  else
    p->_reservation_id = 1;

  p->_last_ready = std::nullopt;
  p->_last_reached = 0;

  p->_send_reservation();
}

//==============================================================================
const std::vector<Writer::Checkpoint>& Participant::path() const
{
  return _pimpl->_shared->_current_reservation.path;
}

//==============================================================================
void Participant::ready(CheckpointId checkpoint)
{
  const auto& p = _pimpl->_shared;
  if (p->_current_reservation.path.size()-1 <= checkpoint)
  {
    // TODO(MXG): Should we consider throwing an exception here?
    checkpoint = p->_current_reservation.path.size() - 2;
  }

  if (p->_last_ready.has_value() && checkpoint <= *p->_last_ready)
    return;

  p->_last_ready = checkpoint;
  p->_send_ready();
}

//==============================================================================
void Participant::release(CheckpointId checkpoint)
{
  const auto& p = _pimpl->_shared;
  if (!p->_last_ready.has_value())
    return;

  if (p->_last_ready.value() < checkpoint)
    return;

  if (checkpoint > 0)
    p->_last_ready = checkpoint - 1;
  else
    p->_last_ready.reset();

  p->_send_release(checkpoint);
}

//==============================================================================
std::optional<CheckpointId> Participant::last_ready() const
{
  return _pimpl->_shared->_last_ready;
}

//==============================================================================
void Participant::reached(CheckpointId checkpoint)
{
  const auto& p = _pimpl->_shared;
  if (checkpoint <= p->_last_reached)
    return;

  p->_last_reached = checkpoint;
  p->_send_reached();
}

//==============================================================================
void Participant::cancel()
{
  const auto& p = _pimpl->_shared;
  if (p->_reservation_id)
  {
    p->_writer->cancel(p->_id, *p->_reservation_id);
    p->_reservation_id = std::nullopt;
  }
}

//==============================================================================
CheckpointId Participant::last_reached() const
{
  return _pimpl->_shared->_last_reached;
}

//==============================================================================
ParticipantId Participant::id() const
{
  return _pimpl->_shared->_id;
}

//==============================================================================
std::optional<ReservationId> Participant::reservation_id() const
{
  return _pimpl->_shared->_reservation_id;
}

//==============================================================================
Participant::Participant()
{
  // Do nothing
}

//==============================================================================
Participant make_participant(
  const ParticipantId participant_id,
  const double radius,
  std::shared_ptr<Writer> writer,
  std::shared_ptr<RectificationRequesterFactory> rectifier_factory)
{
  if (!writer)
  {
    throw std::runtime_error(
            "[rmf_traffic::blockade::make_participant] A nullptr was given for "
            "the `writer` argument. This is illegal.");
  }

  return Participant::Implementation::make(
    participant_id, radius, std::move(writer),
    std::move(rectifier_factory));
}

} // namespace blockade
} // namespace rmf_traffic
