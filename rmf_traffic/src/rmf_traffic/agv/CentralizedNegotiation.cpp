/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <rmf_traffic/agv/CentralizedNegotiation.hpp>
#include <rmf_traffic/schedule/Negotiation.hpp>

#include <deque>
#include <iostream>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class CentralizedNegotiation::Agent::Implementation
{
public:

  schedule::ParticipantId id;
  std::vector<Plan::Start> starts;
  Plan::Goal goal;
  std::shared_ptr<const Planner> planner;
  std::optional<SimpleNegotiator::Options> options;

};

//==============================================================================
CentralizedNegotiation::Agent::Agent(
  schedule::ParticipantId id,
  Plan::Start start,
  Plan::Goal goal,
  std::shared_ptr<const Planner> planner,
  std::optional<SimpleNegotiator::Options> options)
: _pimpl(
    rmf_utils::make_impl<Implementation>(
      Implementation{
        id, {std::move(start)}, std::move(goal),
        std::move(planner), std::move(options)
      }))
{
  // Do nothing
}

//==============================================================================
CentralizedNegotiation::Agent::Agent(
  schedule::ParticipantId id,
  std::vector<Plan::Start> starts,
  Plan::Goal goal,
  std::shared_ptr<const Planner> planner,
  std::optional<SimpleNegotiator::Options> options)
: _pimpl(
    rmf_utils::make_impl<Implementation>(
      Implementation{
        id, std::move(starts), std::move(goal),
        std::move(planner), std::move(options)
      }))
{
  // Do nothing
}

//==============================================================================
schedule::ParticipantId CentralizedNegotiation::Agent::id() const
{
  return _pimpl->id;
}

//==============================================================================
auto CentralizedNegotiation::Agent::id(schedule::ParticipantId value) -> Agent&
{
  _pimpl->id = value;
  return *this;
}

//==============================================================================
const std::vector<Plan::Start>& CentralizedNegotiation::Agent::starts() const
{
  return _pimpl->starts;
}

//==============================================================================
auto CentralizedNegotiation::Agent::starts(std::vector<Plan::Start> values)
-> Agent&
{
  _pimpl->starts = std::move(values);
  return *this;
}

//==============================================================================
const Plan::Goal& CentralizedNegotiation::Agent::goal() const
{
  return _pimpl->goal;
}

//==============================================================================
auto CentralizedNegotiation::Agent::goal(Plan::Goal value) -> Agent&
{
  _pimpl->goal = std::move(value);
  return *this;
}

//==============================================================================
const std::shared_ptr<const Planner>&
CentralizedNegotiation::Agent::planner() const
{
  return _pimpl->planner;
}

//==============================================================================
auto CentralizedNegotiation::Agent::planner(
  std::shared_ptr<const Planner> value) -> Agent&
{
  _pimpl->planner = std::move(value);
  return *this;
}

//==============================================================================
const std::optional<SimpleNegotiator::Options>&
CentralizedNegotiation::Agent::options() const
{
  return _pimpl->options;
}

//==============================================================================
auto CentralizedNegotiation::Agent::options(
  std::optional<SimpleNegotiator::Options> value) -> Agent&
{
  _pimpl->options = std::move(value);
  return *this;
}

//==============================================================================
class CentralizedNegotiation::Result::Implementation
{
public:

  std::optional<Proposal> proposal;
  std::unordered_set<schedule::ParticipantId> blockers;
  std::vector<std::string> log;

  static Result make()
  {
    Result output;
    output._pimpl = rmf_utils::make_impl<Implementation>();
    return output;
  }

  static Implementation& get(Result& result)
  {
    return *result._pimpl;
  }
};

//==============================================================================
auto CentralizedNegotiation::Result::proposal() const
-> const std::optional<Proposal>&
{
  return _pimpl->proposal;
}

//==============================================================================
const std::unordered_set<schedule::ParticipantId>&
CentralizedNegotiation::Result::blockers() const
{
  return _pimpl->blockers;
}

//==============================================================================
const std::vector<std::string>& CentralizedNegotiation::Result::log() const
{
  return _pimpl->log;
}

//==============================================================================
class CentralizedNegotiation::Implementation
{
public:
  std::shared_ptr<const schedule::Viewer> viewer;
  bool optimal = false;
  bool log = false;
  bool print = false;
};

//==============================================================================
CentralizedNegotiation::CentralizedNegotiation(
  std::shared_ptr<const schedule::Viewer> viewer)
: _pimpl(
    rmf_utils::make_impl<Implementation>(Implementation{std::move(viewer)}))
{
  // Do nothing
}

//==============================================================================
const std::shared_ptr<const schedule::Viewer>&
CentralizedNegotiation::viewer() const
{
  return _pimpl->viewer;
}

//==============================================================================
CentralizedNegotiation& CentralizedNegotiation::viewer(
  std::shared_ptr<const schedule::Viewer> v)
{
  _pimpl->viewer = std::move(v);
  return *this;
}

//==============================================================================
CentralizedNegotiation& CentralizedNegotiation::optimal(bool on)
{
  _pimpl->optimal = on;
  return *this;
}

//==============================================================================
CentralizedNegotiation& CentralizedNegotiation::log(bool on)
{
  _pimpl->log = on;
  return *this;
}

//==============================================================================
CentralizedNegotiation& CentralizedNegotiation::print(bool on)
{
  _pimpl->print = on;
  return *this;
}

//==============================================================================
namespace {
std::string display_itinerary(const schedule::Itinerary& itinerary)
{
  std::stringstream ss;
  for (const auto& r : itinerary)
  {
    ss << "\n" << r.map() << ": ";
    for (std::size_t i = 0; i < r.trajectory().size(); ++i)
    {
      const auto& wp = r.trajectory()[i];
      if (wp.velocity().norm() > 1e-2)
        continue;

      if (i > 0)
        ss << " -> ";

      const auto t = wp.time().time_since_epoch();
      ss << "t: " << rmf_traffic::time::to_seconds(t);
      ss << ", (" << wp.position().block<2, 1>(0, 0).transpose()
         << "), yaw: " << wp.position()[2];
    }
  }

  return ss.str();
}
} // anonymous namespace

//==============================================================================
auto CentralizedNegotiation::solve(const std::vector<Agent>& agents) const
-> Result
{
  std::deque<schedule::Negotiation::TablePtr> queue;
  std::unordered_map<schedule::ParticipantId, SimpleNegotiator> negotiators;
  std::unordered_map<schedule::ParticipantId, Plan> proposal;

  auto approvals = std::make_shared<schedule::SimpleResponder::ApprovalMap>();
  auto blockers = std::make_shared<schedule::SimpleResponder::BlockerSet>();

  std::vector<schedule::ParticipantId> participants;
  for (const auto& a : agents)
  {
    SimpleNegotiator::Options options;
    if (a.options().has_value())
      options = *a.options();

    using UpdateVersion = SimpleNegotiator::Responder::UpdateVersion;
    options.approval_callback(
      [&proposal, id = a.id()](rmf_traffic::agv::Plan plan) -> UpdateVersion
      {
        proposal.insert_or_assign(id, std::move(plan));
        return std::nullopt;
      });

    const auto inserted = negotiators.insert(
      {
        a.id(),
        SimpleNegotiator(
          std::make_shared<schedule::Participant::AssignIDPtr::element_type>(),
          a.starts(), a.goal(), a.planner(), std::move(options))
      }).second;

    if (!inserted)
    {
      // *INDENT-OFF*
      throw std::runtime_error(
        "[CentralizedNegotiation::solve] Duplicate participant ["
        + std::to_string(a.id()) + "] in list of agents");
      // *INDENT-ON*
    }

    participants.push_back(a.id());
  }

  auto negotiation = schedule::Negotiation::make(_pimpl->viewer, participants);
  if (!negotiation.has_value())
  {
    // *INDENT-OFF*
    throw std::runtime_error(
      "[CentralizedNegotiation::solve] Unable to begin negotiation because "
      "one or more participants are not registered in the schedule.");
    // *INDENT-ON*
  }

  for (const auto& p : negotiation->participants())
  {
    const auto table = negotiation->table(p, {});
    queue.push_back(table);
  }

  auto finished = [&]()
    {
      if (_pimpl->optimal)
        return negotiation->complete();

      return negotiation->ready();
    };

  auto skip = [](const schedule::Negotiation::TablePtr& table)
    {
      if (table->submission() && !table->rejected())
        return true;

      if (table->version() > 2)
        return true;

      auto ancestor = table->parent();
      while (ancestor)
      {
        if (ancestor->rejected() || ancestor->forfeited())
          return true;

        ancestor = ancestor->parent();
      }

      return false;
    };

  auto result = Result::Implementation::make();
  auto& rimpl = Result::Implementation::get(result);
  const auto log_or_print = _pimpl->log || _pimpl->print;
  const auto progress = [&](std::string msg)
    {
      if (_pimpl->print)
        std::cout << msg << std::endl;

      if (_pimpl->log)
        rimpl.log.push_back(std::move(msg));
    };

  const auto selected_table = [](const schedule::Negotiation::TablePtr& table)
    -> std::string
    {
      std::string msg = "Selected table [";
      for (const auto& p : table->sequence())
      {
        msg += " " + std::to_string(p.participant)
          + ":" + std::to_string(p.version);
      }
      msg += " ]";
      return msg;
    };

  while (!queue.empty() && !finished())
  {
    const auto top = queue.back();
    queue.pop_back();

    if (log_or_print)
      progress(selected_table(top));

    if (skip(top))
    {
      if (log_or_print)
        progress("Skipping");

      continue;
    }

    auto& negotiator = negotiators.at(top->participant());

    blockers->clear();
    const auto viewer = top->viewer();
    negotiator.respond(
      viewer, schedule::SimpleResponder::make(top, approvals, blockers));

    if (top->submission())
    {
      if (log_or_print)
        progress("Submitted plan:" + display_itinerary(*top->submission()));

      for (const auto& [p, _] : negotiators)
      {
        const auto respond_to = top->respond(p);
        if (respond_to)
          queue.push_back(respond_to);
      }

      continue;
    }

    const auto parent = top->parent();
    if (parent && parent->rejected())
    {
      if (log_or_print)
        progress("Rejected parent");

      queue.push_front(parent);
    }

    if (top->forfeited())
    {
      if (log_or_print)
        progress("Forfeited");

      for (const auto& b : *blockers)
        rimpl.blockers.insert(b);
    }
  }

  for (const auto& p : participants)
    rimpl.blockers.erase(p);

  if (!negotiation->ready())
    return result;

  auto selection = negotiation->evaluate(schedule::QuickestFinishEvaluator());
  while (selection)
  {
    approvals->at(selection)();
    selection = selection->parent();
  }

  rimpl.proposal = std::move(proposal);
  return result;
}

} // namespace agv
} // namespace rmf_traffic
