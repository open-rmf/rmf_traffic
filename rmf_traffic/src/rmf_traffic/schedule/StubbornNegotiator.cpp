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
void add_offset_itinerary(
  rmf_traffic::Duration offset,
  const std::vector<rmf_traffic::Route>& original,
  std::vector<rmf_traffic::Route>& output)
{
  auto shadow = original;
  for (auto& item : shadow)
  {
    if (item.trajectory().empty())
      continue;

    const auto initial_time = *item.trajectory().start_time();
    item.trajectory().front().adjust_times(offset);
    item.trajectory().insert(
      initial_time,
      item.trajectory().front().position(),
      Eigen::Vector3d::Zero());
  }

  output.insert(output.end(), shadow.begin(), shadow.end());
}

//==============================================================================
std::vector<rmf_traffic::Route> add_margins(
  const std::vector<rmf_traffic::Route>& original,
  const std::vector<rmf_traffic::Duration>& margins)
{
  auto itinerary = original;
  using namespace std::chrono_literals;
  for (const auto t : margins)
    add_offset_itinerary(t, original, itinerary);

  return itinerary;
}

//==============================================================================
class StubbornNegotiator::Implementation
{
public:

  const Participant* participant;
  std::shared_ptr<const Participant> shared_ref;
  std::vector<rmf_traffic::Duration> acceptable_waits;
  std::vector<rmf_traffic::Duration> margins;
  std::function<UpdateVersion(rmf_traffic::Duration)> approval_cb = nullptr;

  std::optional<std::vector<rmf_traffic::Route>> test_candidate(
    rmf_traffic::Duration offset,
    const std::vector<rmf_traffic::Route>& original,
    const rmf_traffic::agv::NegotiatingRouteValidator& validator,
    std::vector<rmf_traffic::schedule::Itinerary>& alternatives) const;
};

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
      alternatives.push_back(candidate);
      return std::nullopt;
    }
  }

  return candidate;
}

//==============================================================================
StubbornNegotiator::StubbornNegotiator(const Participant& participant)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{&participant, nullptr, {}, {}}))
{
  // Do nothing
}

//==============================================================================
StubbornNegotiator::StubbornNegotiator(
  std::shared_ptr<const Participant> participant)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{participant.get(), participant, {}, {}}))
{
  // Do nothing
}

//==============================================================================
StubbornNegotiator& StubbornNegotiator::acceptable_waits(
  std::vector<rmf_traffic::Duration> wait_times,
  std::function<UpdateVersion(rmf_traffic::Duration)> approval_cb)
{
  _pimpl->acceptable_waits = std::move(wait_times);
  _pimpl->approval_cb = std::move(approval_cb);
  return *this;
}

//==============================================================================
StubbornNegotiator& StubbornNegotiator::additional_margins(
  std::vector<rmf_traffic::Duration> margins)
{
  _pimpl->margins = std::move(margins);
  return *this;
}

//==============================================================================
void StubbornNegotiator::respond(
  const schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  using namespace std::chrono_literals;

  const auto& original = _pimpl->participant->itinerary();
  auto generator =
    rmf_traffic::agv::NegotiatingRouteValidator::Generator(
    table_viewer, _pimpl->participant->description().profile())
    .ignore_unresponsive()
    .ignore_bystanders();

  std::vector<rmf_traffic::schedule::Itinerary> alternatives;
  for (const auto& validator : generator.all())
  {
    if (_pimpl->test_candidate(0s, original, *validator, alternatives)
      .has_value())
    {
      responder->submit(
        _pimpl->participant->plan_id_assigner()->assign(),
        add_margins(original, _pimpl->margins),
        [cb = _pimpl->approval_cb]() -> UpdateVersion
        {
          if (cb)
            return cb(0s);

          return std::nullopt;
        });
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
          responder->submit(
            _pimpl->participant->plan_id_assigner()->assign(),
            add_margins(*candidate, _pimpl->margins),
            [
              cb = _pimpl->approval_cb,
              delay = t
            ]() -> UpdateVersion
            {
              if (cb)
                return cb(delay);

              return std::nullopt;
            });
          return;
        }
      }
    }
  }

  if (table_viewer->sequence().back().version < 3)
  {
    if (table_viewer->sequence().size() <= 1)
    {
      // We have been rejected too many times, and we have no parent to reject.
      // Let's just forfeit.
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
