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

#ifndef RMF_TRAFFIC__RESERVATIONS__INTERNAL_PROTOCOL_HPP
#define RMF_TRAFFIC__RESERVATIONS__INTERNAL_PROTOCOL_HPP

#include "ParticipantStore.hpp"
#include "Optimizer.hpp"
#include "State.hpp"
#include "StateDiff.hpp"

namespace rmf_traffic {
namespace reservations {

//==============================================================================
/// \brief The Protocol class is in charge of handling the requests and
/// negotiations that occur during the planning and negotiation phase.
class Protocol
{
public:
  //============================================================================
  /// \brief Constructor. Starts an execution thread engine which consumes
  /// commands of the incoming participants.
  /// \param[in] request_queue - the `RequestQueue` upon which you will send
  ///   commands.
  /// \param[in] participant_store - the `ParticipantStore` containing the list
  ///   of participants.
  Protocol(
    std::shared_ptr<ParticipantStore> pstore)
  : _participant_store(pstore),
    _curr_state(std::make_shared<RequestStore>()),
    _proposal_version(0)
  {

  }

  //============================================================================
  /// \brief Destructor. Terminates the execution engine by sending a
  /// TERMINATE_STREAM action and waits for the destructor to join.
  ~Protocol()
  {

  }

  //============================================================================
  /// \brief Add a request to the queue.
  void add_request(
    ParticipantId participant_id,
    RequestId request_id,
    std::vector<ReservationRequest>& reservation,
    int priority)
  {
    _curr_state = _curr_state.add_request(
      participant_id,
      request_id,
      priority,
      reservation
    );

    auto heuristic = std::make_shared<PriorityBasedScorer>();

    GreedyBestFirstSearchOptimizer optimizer(heuristic);
    auto favored_solution = get_favored_solution(optimizer);
    if (!favored_solution.has_value())
      return;

    rollout(favored_solution.value());
  }

  //============================================================================
  /// \brief Remove a request
  void remove_request(
    ParticipantId participant_id,
    RequestId request_id)
  {
    _curr_state = _curr_state.remove_request(
      participant_id, request_id);

    auto heuristic = std::make_shared<PriorityBasedScorer>();

    GreedyBestFirstSearchOptimizer optimizer(heuristic);
    auto favored_solution = get_favored_solution(optimizer);
    if (!favored_solution.has_value())
      return;

    rollout(favored_solution.value());
  }

  //============================================================================
  /// \brief Remove a participant
  void remove_participant(ParticipantId participant_id)
  {
    _curr_state = _curr_state.remove_participant(participant_id);
    _participant_store->remove_participant(participant_id);

    auto heuristic = std::make_shared<PriorityBasedScorer>();

    GreedyBestFirstSearchOptimizer optimizer(heuristic);
    auto favored_solution = get_favored_solution(optimizer);
    if (!favored_solution.has_value())
      return;

    rollout(favored_solution.value());
  }

private:
  //============================================================================
  /// \brief Rolls out the proposed changes.
  /// TODO: Consider making asynchronous
  void rollout(const StateDiff& changes)
  {
    for (auto change: changes.differences())
    {
      if (!_participant_store->get_participant(change.participant_id).has_value())
      {
        continue;
      }

      auto participant =
        _participant_store->get_participant(change.participant_id).value();
      if (change.diff_type == StateDiff::DifferenceType::SHIFT_RESERVATION)
      {
        assert(change.reservation.has_value());

        auto result = participant->request_confirmed(
          change.request_id,
          change.reservation.value(),
          _proposal_version);

        if (result == false)
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
      else if (
        change.diff_type == StateDiff::DifferenceType::ASSIGN_RESERVATION)
      {
        assert(change.reservation.has_value());

        auto result = participant->request_confirmed(
          change.request_id,
          change.reservation.value(),
          _proposal_version);

        if (result == false)
          return;

        _curr_state.assign_reservation(
          change.participant_id,
          change.request_id,
          change.reservation.value()
        );
      }
      else if (
        change.diff_type == StateDiff::DifferenceType::UNASSIGN_RESERVATION)
      {
        assert(!change.reservation.has_value());
        auto result =
          participant->unassign_request_confirmed(
          change.request_id,
          _proposal_version);

        if (result == false)
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
  /// TODO(arjo): async-ify
  /// \param[in] optimizer - Instance of the optimizer.
  /// \returns the StateDiff or a `std::nullopt` if no Solution is agreeable
  std::optional<StateDiff> get_favored_solution(
    GreedyBestFirstSearchOptimizer& optimizer)
  {
    auto solutions = optimizer.optimize(_curr_state);
    while (auto solution = solutions.next_solution())
    {
      StateDiff impacted_reservations(solution.value(), _curr_state);
      if (verify_proposal(impacted_reservations))
        return {impacted_reservations};
      _proposal_version++;
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
    for (auto diff: res)
    {
      auto impacted_participant = diff.participant_id;

      if(!_participant_store->get_participant(impacted_participant).has_value())
        continue;

      auto participant =
        _participant_store->get_participant(impacted_participant)
        .value();

      if (diff.diff_type == StateDiff::DifferenceType::ASSIGN_RESERVATION
        || diff.diff_type == StateDiff::DifferenceType::SHIFT_RESERVATION)
      {
        auto result = participant->request_proposal(
          diff.request_id,
          diff.reservation.value(),
          _proposal_version
        );
        if (!result)
          return false;
      }
      else
      {
        auto result = participant->unassign_request_proposal(
          diff.request_id,
          _proposal_version
        );
        if (!result)
          return false;
      }
    }
    return true;
  }

  std::shared_ptr<ParticipantStore> _participant_store;
  uint64_t _proposal_version;
  State _curr_state;
};
}
}

#endif
