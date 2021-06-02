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

#include <rmf_traffic/reservations/Database.hpp>
#include "internal/ParticipantStore.hpp"
#include "internal/RequestQueue.hpp"
#include "internal/State.hpp"
namespace rmf_traffic {
namespace reservations {

class Database::Implementation
{
public:
  Implementation():
    _request_queue(std::make_shared<RequestQueue>())
  {

  }

  void register_participant(
    ParticipantId id,
    std::shared_ptr<Participant> participant)
  {
    _participant_store.add_participant(id, participant);
  }

  void unregister_participant(
    ParticipantId id
  )
  {
    _participant_store.remove_participant(id);
    _request_queue->erase_participant_requests(id);
  }

  void request_reservation(
    ParticipantId id,
    RequestId req,
    std::vector<ReservationRequest>& request_options,
    int priority)
  {

  }

  void cancel_request(ParticipantId id, RequestId req)
  {
  }
private:
  ParticipantStore _participant_store;
  std::shared_ptr<RequestQueue> _request_queue;
};

void Database::register_participant(
  ParticipantId id,
  std::shared_ptr<Participant> participant)
{
  _pimpl->register_participant(
    id,
    participant
  );
}

void Database::unregister_participant(ParticipantId id)
{
  _pimpl->unregister_participant(id);
}

void Database::request_reservation(
  ParticipantId id,
  RequestId req,
  std::vector<ReservationRequest>& request_options,
  int priority)
{
  _pimpl->request_reservation(id, req, request_options, priority);
}

void Database::cancel_request(ParticipantId id, RequestId req)
{
  _pimpl->cancel_request(id, req);
}
Database::Database():
  _pimpl(rmf_utils::make_impl<Implementation>())
{
}

}
}