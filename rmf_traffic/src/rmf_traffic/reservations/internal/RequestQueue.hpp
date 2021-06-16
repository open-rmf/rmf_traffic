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

class RequestQueue
{
public:
  enum ActionType
  {
    ADD,
    REMOVE,
    REMOVE_PARTICIPANT
  };
  struct Action
  {
    ActionType type;
    ParticipantId participant_id;
    RequestId request_id;
    std::vector<ReservationRequest>& request_options;
    int priority;
  };

  struct QueueElement
  {
    Action action;
    std::shared_ptr<RequestStore> request_store;
  };

  void add_action(Action action)
  {
    {
      std::lock_guard lock(_mutex);

      std::shared_ptr<RequestStore>
        req_store = [=]() -> std::shared_ptr<RequestStore>
        {
          if(_request_store_queue.empty())
            return std::make_shared<RequestStore>();
          else
            return std::make_shared<RequestStore>(
              _request_store_queue.back().request_store.get()
            );
        }();

      if(action.type == ActionType::ADD)
      {
        req_store->enqueue_reservation(
          action.participant_id,
          action.request_id,
          action.priority,
          action.request_options);
      }
      else if(action.type == ActionType::REMOVE)
      {
        req_store->cancel(action.participant_id, action.request_id);
      }
      else
      {
        req_store->erase_participant_requests(action.participant_id);
      }
      _request_store_queue.push_back({action, req_store});
    }
    _cond.notify_one();
  }

  QueueElement deque()
  {
    std::unique_lock loc(_mutex);
    _cond.wait(loc, [=](){return !_request_store_queue.empty();});
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