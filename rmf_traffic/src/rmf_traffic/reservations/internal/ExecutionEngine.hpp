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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_EXECUTIONENGINE_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_EXECUTIONENGINE_HPP

#include <thread>

#include "RequestQueue.hpp"
#include "ParticipantStore.hpp"
#include "Optimizer.hpp"
#include "State.hpp"
#include "StateDiff.hpp"

namespace rmf_traffic {
namespace reservations {

/// \brief The execution engine class is in charge of handling the requests and
/// negotiations that occur during the planning and negotiation phase. As such,
/// it maintains a single thread that handles both planning and negotiations
/// while enqueueing requests upon a reservation queue.
class ExecutionEngine
{
  ExecutionEngine(
    std::shared_ptr<RequestQueue> request_queue,
    std::shared_ptr<ParticipantStore> pstore):
      _request_queue(request_queue),
      _participant_store(pstore),
      _execution_thread(&ExecutionEngine::execute, this),
      _curr_state(std::make_shared<RequestStore>())
  {
    //Do nothing
  }

  ~ExecutionEngine()
  {
    std::vector<ReservationRequest> res;
    // Terminate the execution thread
    RequestQueue::Action terminate_req {
      RequestQueue::ActionType::TERMINATE_STREAM,
      0,
      0,
      res,
      0
    };
    _request_queue->add_action(terminate_req);

    if(_execution_thread.joinable())
      _execution_thread.join();
  }

private:
  void execute()
  {
    while(true)
    {
      auto element = _request_queue->deque();

      if(element.action.type == RequestQueue::ActionType::TERMINATE_STREAM)
        return;

      if(element.action.type == RequestQueue::ActionType::ADD)
      {
        _curr_state = _curr_state.add_request(
          element.action.participant_id,
          element.action.request_id,
          element.request_store
        );
      }
      else if(element.action.type == RequestQueue::ActionType::REMOVE)
      {
        _curr_state = _curr_state.remove_request(
          element.action.participant_id,
          element.action.request_id,
          element.request_store
        );
      }
      // TODO(URGENT): Add participant removal

      //TODO: Breakout optimizer
      auto heuristic = std::make_shared<PriorityBasedScorer>(
        element.request_store);
      GreedyBestFirstSearchOptimizer optimizer(heuristic);
      auto solutions = optimizer.optimize(_curr_state);
      while(auto solution = solutions.next_solution())
      {
        StateDiff impacted_reservations(solution.value(), _curr_state);
        auto res = impacted_reservations.differences();
        for(auto diff: res)
        {
          auto impacted_participant = diff.participant_id;
          auto participant =
            _participant_store->get_participant(impacted_participant)
              .value();

          if(diff.diff_type == StateDiff::DifferenceType::ASSIGN_RESERVATION
            || diff.diff_type == StateDiff::DifferenceType::SHIFT_RESERVATION)
          {
            auto result = participant->request_proposal(
              diff.request_id,
              diff.reservation.value()
            );
            if(!result)
              continue;
          }
          else
          {
            auto result = participant->(
              diff.request_id,
              diff.reservation.value()
            );
            if(!result)
              continue;
          }
        }
        break;
      }
    }
  }
  std::thread _execution_thread;
  std::shared_ptr<RequestQueue> _request_queue;
  std::shared_ptr<ParticipantStore> _participant_store;
  State _curr_state;
};
}
}

#endif
