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
#include <rmf_traffic/agv/RouteValidator.hpp>

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class StubbornNegotiator::Implementation
{
public:

  const Participant* participant;
  std::shared_ptr<const Participant> shared_ref;
  std::vector<rmf_traffic::Duration> acceptable_waits;

  std::optional<std::vector<rmf_traffic::Route>> test_candidate(
    rmf_traffic::Duration offset,
    const std::vector<rmf_traffic::Route>& original,
    const rmf_traffic::agv::NegotiatingRouteValidator& validator,
    std::vector<rmf_traffic::schedule::Itinerary>& alternatives) const;
};

//==============================================================================
namespace {
Itinerary move_to_itinerary(std::vector<rmf_traffic::Route>&& candidate)
{
  Itinerary output;
  output.reserve(candidate.size());
  for (auto&& r : std::move(candidate))
  {
    output.push_back(std::make_shared<rmf_traffic::Route>(std::move(r)));
  }

  return output;
}
} // anonymous namespace

//==============================================================================
std::optional<std::vector<Route>>
StubbornNegotiator::Implementation::test_candidate(
  rmf_traffic::Duration offset,
  const std::vector<rmf_traffic::Route>& original,
  const rmf_traffic::agv::NegotiatingRouteValidator& validator,
  std::vector<rmf_traffic::schedule::Itinerary>& alternatives) const
{
  using namespace std::chrono_literals;
  auto candidate = original;
  if (offset != 0s)
  {
    for (auto& r : candidate)
    {
      auto& traj = r.trajectory();
      if (!traj.empty())
        traj.front().adjust_times(offset);
    }
  }

  for (const auto& r : candidate)
  {
    if (validator.find_conflict(r))
    {
      alternatives.push_back(move_to_itinerary(std::move(candidate)));
      return std::nullopt;
    }
  }

  return candidate;
}

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
  using namespace std::chrono_literals;

  std::vector<rmf_traffic::Route> original;
  const auto& itinerary = _pimpl->participant->itinerary();
  for (const auto& item : itinerary)
    original.push_back(*item.route);

  auto generator =
    rmf_traffic::agv::NegotiatingRouteValidator::Generator(
      table_viewer, _pimpl->participant->description().profile());

  std::vector<rmf_traffic::schedule::Itinerary> alternatives;
  for (const auto& validator : generator.all())
  {
    if (_pimpl->test_candidate(0s, original, *validator, alternatives)
        .has_value())
    {
      responder->submit(original);
      return;
    }

    if (table_viewer->rejected())
    {
      for (const auto& t : _pimpl->acceptable_waits)
      {
        const auto candidate =
          _pimpl->test_candidate(t, original, *validator, alternatives);
        if (candidate.has_value())
        {
          responder->submit(original);
          return;
        }
      }
    }
  }

  if (table_viewer->sequence().back().version < 3)
  {
    if (table_viewer->sequence().size() <= 1)
    {
      // If this is the first participant in the negotiation sequence, then we
      // need to forfeit, because there is no lower level proposal to reject.
      responder->forfeit({});
    }

    responder->reject(alternatives);
  }
  else
  {
    // If we have had multiple rejections, then let's just forfeit.
    responder->forfeit({});
  }
}

} // namespace schedule
} // namespace rmf_traffic
