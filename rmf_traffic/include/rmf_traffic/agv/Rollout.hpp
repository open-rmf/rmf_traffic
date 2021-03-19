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

#ifndef RMF_TRAFFIC__AGV__ROLLOUT_HPP
#define RMF_TRAFFIC__AGV__ROLLOUT_HPP

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// The Rollout class complements the Planner class. The Planner class may
/// sometimes fail to find a feasible plan because of other traffic participants
/// blocking the way. The rollout class can take a Planner::Result and expand
/// sets of alternative routes that would be feasible in the absence of a
/// blocker. Given these sets of alternatives,
class Rollout
{
public:

  /// Constructor
  ///
  /// \param[in] result
  ///   The Planning Result that should be rolled out.
  Rollout(Planner::Result result);

  /// Expand the Planning Result through the specified blocker.
  ///
  /// \param[in] blocker
  ///   The blocking participant that should be expanded through. If this
  ///   participant wasn't actually blocking, then the returned vector will be
  ///   empty.
  ///
  /// \param[in] span
  ///   How far into the future the rollout should continue. Once a rollout
  ///   extends this far, it will stop wherever it is.
  ///
  /// \param[in] options
  ///   The options to use while expanding. NOTE: It is important to provide a
  ///   RouteValidator that will ignore the blocker, otherwise the expansion
  ///   might not give back any useful results.
  ///
  /// \param[in] max_rollouts
  ///   The maximum number of rollouts to produce.
  ///
  /// \return a collection of itineraries from the original Planning Result's
  /// starts past the blockages that were caused by the specified blocker.
  std::vector<schedule::Itinerary> expand(
    schedule::ParticipantId blocker,
    rmf_traffic::Duration span,
    const Planner::Options& options,
    rmf_utils::optional<std::size_t> max_rollouts = rmf_utils::nullopt) const;

  /// Expand the Planning Result through the specified behavior. Use the Options
  /// that are already tied to the Planning Result.
  ///
  /// \warning It is critical to change the validator in the Planner Result
  /// Options before giving it to the Rollout if you want to use this method.
  /// Otherwise there will not be any expansion through the blocker.
  ///
  /// \param[in] blocker
  ///   The blocking participant that should be expanded through. If this
  ///   participant wasn't actually blocking, then the returned vector will be
  ///   empty.
  ///
  /// \param[in] span
  ///   How far into the future the rollout should continue. Once a rollout
  ///   extends this far, it will stop wherever it is.
  ///
  /// \param[in] max_rollouts
  ///   The maximum number of rollouts to produce.
  ///
  /// \return a collection of itineraries from the original Planning Result's
  /// starts past the blockages that were caused by the specified blocker.
  std::vector<schedule::Itinerary> expand(
    schedule::ParticipantId blocker,
    rmf_traffic::Duration span,
    rmf_utils::optional<std::size_t> max_rollouts = rmf_utils::nullopt) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__ROLLOUT_HPP
