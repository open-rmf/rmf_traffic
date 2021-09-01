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

#ifndef RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP
#define RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP

#include <rmf_traffic/schedule/Negotiator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
/// A StubbornNegotiator will only accept plans that accommodate the current
/// itinerary of the
class StubbornNegotiator : public Negotiator
{
public:

  /// Constructor
  ///
  /// \note We take a const-reference to the Participant with the expectation
  /// that the Participant instance will outlive this StubbornNegotiator
  /// instance. The StubbornNegotiator costs very little to construct, so it is
  /// okay to use a pattern like
  ///
  /// \code
  /// StubbornNegotiator(participant).respond(table_view, responder);
  /// \endcode
  ///
  /// \param[in] participant
  ///   The Participant who wants to be stubborn.
  StubbornNegotiator(const Participant& participant);

  /// Owning Constructor
  ///
  /// The StubbornNegotiator instance will now hold a shared reference to the
  /// participant to ensure it maintains its lifetime. This constructor should
  /// be used in cases where the StubbornNegotiator instance has a prolonged
  /// lifecycle.
  ///
  /// \param[in] participant
  ///   The Participant who wants to be stubborn.
  StubbornNegotiator(std::shared_ptr<const Participant> participant);

  using UpdateVersion = rmf_utils::optional<ItineraryVersion>;

  /// Add a set of acceptable wait times.
  ///
  /// \param[in] wait_times
  ///   A list of the wait times that would be accepted for negotiation
  ///
  /// \param[in] approval_cb
  ///   A callback that will be triggered when the negotiator decides that you
  ///   need to wait for another participant. The callback will receive the
  ///   chosen wait duration, and is expected to return the schedule version
  ///   that will incorporate the given wait time.
  StubbornNegotiator& acceptable_waits(
    std::vector<Duration> wait_times,
    std::function<UpdateVersion(Duration wait_time)> approval_cb = nullptr);

  /// Add some timing margins that will be put into the negotiation submission.
  /// This effectively asks other robots to back off somewhat.
  ///
  /// \param[in] margins
  ///   The margins to put into the proposal.
  StubbornNegotiator& additional_margins(
    std::vector<rmf_traffic::Duration> margins);

  void respond(
    const schedule::Negotiation::Table::ViewerPtr& table_viewer,
    const ResponderPtr& responder) final;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__SCHEDULE__STUBBORNNEGOTIATOR_HPP
