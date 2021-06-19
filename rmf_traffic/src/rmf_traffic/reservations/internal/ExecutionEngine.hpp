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

//==============================================================================
/// \brief The execution engine class is in charge of handling the requests and
/// negotiations that occur during the planning and negotiation phase. As such,
/// it maintains a single thread that handles both planning and negotiations
/// while enqueueing requests upon a reservation queue. This is where the crux
/// of the protocol is implemented
class ExecutionEngine
{
  //============================================================================
  /// \brief Constructor. Starts an execution thread engine which consumes
  /// commands of the incoming participants.
  /// \param[in] request_queue - the `RequestQueue` upon which you will send
  ///   commands.
  /// \param[in] participant_store - the `ParticipantStore` containing the list
  ///   of participants.
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

  //============================================================================
  /// \brief Destructor. Terminates the execution engine by sending a
  /// TERMINATE_STREAM action and waits for the destructor to join.
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
  //============================================================================
  /// \brief Main thread: blocks until a request has come in. When a request
  /// comes in the current_state is read and updated to contain the request.
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
      else if(element.action.type == RequestQueue::ActionType::REMOVE_PARTICIPANT)
      {
        _curr_state = _curr_state.remove_participant(
          element.action.participant_id,
          element.request_store);
      }

      auto heuristic = std::make_shared<PriorityBasedScorer>(
        element.request_store);

      GreedyBestFirstSearchOptimizer optimizer(heuristic);
      auto favored_solution = get_favored_solution(optimizer);
      if(!favored_solution.has_value())
        continue;

      rollout(favored_solution.value());

    }
  }

  //============================================================================
  /// \brief Rolls out the changes. If for whatever reason a rollout should fail
  /// there is no need to rollback as the StateDiff is returned in a topological
  /// order. Instead _`curr_state` remains in a partially successful state and
  /// we will retry optimization upon the arrival of the next request.
  void rollout(const StateDiff& changes)
  {
    for(auto change: changes.differences())
    {
      assert(
        _participant_store->get_participant(change.participant_id).has_value());

      auto participant =
        _participant_store->get_participant(change.participant_id).value();
      if(change.diff_type == StateDiff::DifferenceType::SHIFT_RESERVATION)
      {
        assert(change.reservation.has_value());

        auto result = participant->request_confirmed(
          change.request_id,
          change.reservation.value());

        if(result == false)
          return;

        _curr_state = _curr_state.unassign_reservation(
          change.participant_id,
          change.request_id);

        _curr_state.assign_reservation(
          change.participant_id,
          change.request_id,
          change.reservation.value()
        );
      }
      else if(
        change.diff_type == StateDiff::DifferenceType::ASSIGN_RESERVATION)
      {
        assert(change.reservation.has_value());

        auto result = participant->request_confirmed(
          change.request_id,
          change.reservation.value());

        if(result == false)
          return;

        _curr_state.assign_reservation(
          change.participant_id,
          change.request_id,
          change.reservation.value()
        );
      }
      else if(
        change.diff_type == StateDiff::DifferenceType::UNASSIGN_RESERVATION)
      {
        assert(!change.reservation.has_value());
        auto result =
          participant->unassign_request_confirmed(change.request_id);

        if(result == false)
          return;

        _curr_state = _curr_state.unassign_reservation(
          change.participant_id,
          change.request_id);
      }
    }
  }

  //============================================================================
  /// \brief Proposes the solution favoured by the optimizer and ratified
  /// by all participants.
  /// \param[in] optimizer - Instance of the optimizer.
  /// \returns the StateDiff or a `std::nullopt` if no Solution is agreeable
  std::optional<StateDiff> get_favored_solution(
    GreedyBestFirstSearchOptimizer& optimizer)
  {
    auto solutions = optimizer.optimize(_curr_state);
    while(auto solution = solutions.next_solution())
    {
      StateDiff impacted_reservations(solution.value(), _curr_state);
      if(verify_proposal(impacted_reservations))
        return {impacted_reservations};
    }

    return std::nullopt;
  }

  //============================================================================
  /// \brief Verifies individual proposals.
  /// \param[in] impacted_reservations -  The proposed changes
  /// \return true if all impacted participants agree otherwise return false
  bool verify_proposal(const StateDiff& impacted_reservations)
  {
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
          return false;
      }
      else
      {
        auto result = participant->unassign_request_proposal(
          diff.request_id
        );
        if(!result)
          return false;
      }
    }
    return true;
  }

  std::thread _execution_thread;
  std::shared_ptr<RequestQueue> _request_queue;
  std::shared_ptr<ParticipantStore> _participant_store;
  State _curr_state;
};
}
}

#endif
