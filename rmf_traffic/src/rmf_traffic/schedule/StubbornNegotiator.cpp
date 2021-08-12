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

#include <rmf_traffic/schedule/StubbornNegotiator.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class StubbornNegotiator::Implementation
{
public:

  const Participant* participant;
  std::shared_ptr<const Participant> shared_ref;
  std::vector<rmf_traffic::Duration> acceptable_waits;

};

//==============================================================================
StubbornNegotiator::StubbornNegotiator(const Participant& participant)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{&participant, nullptr, {}}))
{
  // Do nothing
}

//==============================================================================
StubbornNegotiator::StubbornNegotiator(
  std::shared_ptr<const Participant> participant)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{participant.get(), participant, {}}))
{
  // Do nothing
}

//==============================================================================
StubbornNegotiator& StubbornNegotiator::acceptable_waits(
  std::vector<rmf_traffic::Duration> wait_times)
{
  _pimpl->acceptable_waits = std::move(wait_times);
  return *this;
}

//==============================================================================
void StubbornNegotiator::respond(
  const schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  std::vector<rmf_traffic::Route> submission;

  const auto& itinerary = _pimpl->participant->itinerary();
  for (const auto& item : itinerary)
    submission.push_back(*item.route);

  const auto table_version = table_viewer->sequence().back().version;
  if (table_viewer->rejected() && table_version > 0)
  {
    const auto wait_index = table_version - 1;
    if (wait_index < _pimpl->acceptable_waits.size())
    {
      const auto wait = _pimpl->acceptable_waits[wait_index];
      for (auto& item : submission)
        item.trajectory().front().adjust_times(wait);

      const auto& first_wp = submission.front().trajectory().front();
      submission.front().trajectory().insert(
        first_wp.time() - wait,
        first_wp.position(),
        Eigen::Vector3d::Zero());
    }
    else
    {
      // If we can't come up with an itinerary that hasn't been rejected, then we
      // must forfeit.
      return responder->forfeit({});
    }
  }

  const auto& profile = _pimpl->participant->description().profile();
  const auto& proposal = table_viewer->base_proposals();

  for (const auto& p : proposal)
  {
    const auto other_participant = table_viewer->get_description(p.participant);
    assert(other_participant);
    const auto& other_profile = other_participant->profile();

    for (const auto& other_route : p.itinerary)
    {
      for (const auto& item : submission)
      {
        if (item.map() != other_route->map())
          continue;

        if (rmf_traffic::DetectConflict::between(
            profile,
            item.trajectory(),
            other_profile,
            other_route->trajectory()))
        {
          rmf_traffic::schedule::Itinerary alternative;
          alternative.reserve(submission.size());
          for (const auto& item : submission)
          {
            alternative.emplace_back(
              std::make_shared<rmf_traffic::Route>(item));
          }

          return responder->reject({std::move(alternative)});
        }
      }
    }
  }

  responder->submit(std::move(submission));
}

} // namespace schedule
} // namespace rmf_traffic
