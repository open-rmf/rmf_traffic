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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTQUEUE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_REQUESTQUEUE_HPP

#include <rmf_traffic/reservations/ReservationRequest.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/Participant.hpp>
#include <unordered_map>
#include <deque>
#include <mutex>
#include <condition_variable>
#include "RequestStore.hpp"

namespace rmf_traffic {
namespace reservations {

/// \brief The RequestQueue is used to coordinate dispatch to a the
/// ExecutionEngine.
class RequestQueue
{
public:
  enum ActionType
  {
    /// \brief Add a request
    ADD,
    /// \brief Remove a request
    REMOVE,
    /// \brief Remove an entire participant
    REMOVE_PARTICIPANT,
    /// \brief Terminates the execution engine. Only the ExecutionEngine class
    /// should use this.
    TERMINATE_STREAM
  };
  /// This represents the action which you wish to execute.
  struct Action
  {
    ActionType type;
    ParticipantId participant_id;
    RequestId request_id;
    std::vector<ReservationRequest> request_options;
    int priority;
  };
  struct QueueElement
  {
    Action action;

  };

  void add_action(Action action)
  {
    {
      std::lock_guard lock(_mutex);
      _request_store_queue.push_back({action});
    }
    _cond.notify_one();
  }

  QueueElement deque()
  {
    std::unique_lock loc(_mutex);
    _cond.wait(loc, [=]() {return !_request_store_queue.empty();});
    auto front = _request_store_queue.front();
    _request_store_queue.pop_front();
    loc.unlock();
    return front;
  }

private:
  std::deque<QueueElement> _request_store_queue;
  std::mutex _mutex;
  std::condition_variable _cond;
};

}
}

#endif