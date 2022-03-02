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

#ifndef RMF_TRAFFIC__AGV__CENTRALIZEDNEGOTIATION_HPP
#define RMF_TRAFFIC__AGV__CENTRALIZEDNEGOTIATION_HPP

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/SimpleNegotiator.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class CentralizedNegotiation
{
public:

  class Agent
  {
  public:

    /// Constructor
    ///
    /// \param[in] id
    ///   This agent's ID within the schedule database. If multiple agents are
    ///   given the same ID in a negotiation, then a runtime exception will be
    ///   thrown.
    ///
    /// \param[in] starts
    ///   The starting condition for this agent.
    ///
    /// \param[in] goal
    ///   The goal for this agent.
    ///
    /// \param[in] planner
    ///   The single-agent planner used for this agent. Each agent can have its
    ///   own planner or they can share planners. If this is set to nullptr when
    ///   the negotiation begins, then a runtime exception will be thrown.
    ///
    /// \param[in] options
    ///   Options to use for the negotiator of this agent. If nullopt is
    ///   provided, then the default options of the SimpleNegotiator will be
    ///   used.
    Agent(
      schedule::ParticipantId id,
      Plan::Start start,
      Plan::Goal goal,
      std::shared_ptr<const Planner> planner,
      std::optional<SimpleNegotiator::Options> options = std::nullopt);

    /// Constructor
    ///
    /// \param[in] id
    ///   This agent's ID within the schedule database. If multiple agents are
    ///   given the same ID in a negotiation, then a runtime exception will be
    ///   thrown.
    ///
    /// \param[in] starts
    ///   One or more starting conditions for this agent. If no starting
    ///   conditions are provided before the negotiation begins, then a runtime
    ///   exception will be thrown.
    ///
    ///   The planner will use whichever starting condition provides the optimal
    ///   plan.
    ///
    /// \param[in] goal
    ///   The goal for this agent.
    ///
    /// \param[in] planner
    ///   The single-agent planner used for this agent. Each agent can have its
    ///   own planner or they can share planners. If this is set to nullptr when
    ///   the negotiation begins, then a runtime exception will be thrown.
    ///
    /// \param[in] options
    ///   Options to use for the negotiator of this agent. If nullopt is
    ///   provided, then the default options of the SimpleNegotiator will be
    ///   used.
    Agent(
      schedule::ParticipantId id,
      std::vector<Plan::Start> starts,
      Plan::Goal goal,
      std::shared_ptr<const Planner> planner,
      std::optional<SimpleNegotiator::Options> options = std::nullopt);

    /// Get the ID for this agent
    schedule::ParticipantId id() const;

    /// Set the ID for this agent
    Agent& id(schedule::ParticipantId value);

    /// Get the starts for this agent
    const std::vector<Plan::Start>& starts() const;

    /// Set the starts for this agent
    Agent& starts(std::vector<Plan::Start> values);

    /// Get the goal for this agent
    const Plan::Goal& goal() const;

    /// Set the goal for this agent
    Agent& goal(Plan::Goal value);

    /// Get the planner for this agent
    const std::shared_ptr<const Planner>& planner() const;

    /// Set the planner for this agent
    Agent& planner(std::shared_ptr<const Planner> value);

    /// Get the options for this agent
    const std::optional<SimpleNegotiator::Options>& options() const;

    /// Set the options for this agent
    Agent& options(std::optional<SimpleNegotiator::Options> value);

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// When a proposal is found, it will provide a plan for each agent
  using Proposal = std::unordered_map<schedule::ParticipantId, Plan>;

  class Result
  {
  public:

    /// If a solution was found, it will be provided by this proposal.
    const std::optional<Proposal>& proposal() const;

    /// This is a list of schedule Participants that were not part of the
    /// negotiation who blocked the planning effort. Blockers do not necessarily
    /// prevent a solution from being found, but they do prevent the optimal
    /// solution from being available.
    const std::unordered_set<schedule::ParticipantId>& blockers() const;

    /// A log of messages related to the negotiation. This will be empty unless
    /// the log() function of the CentralizedNegotiation is toggled on before
    /// solving.
    const std::vector<std::string>& log() const;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// Constructor
  ///
  /// \param[in] viewer
  ///   A viewer for the traffic schedule. You may provide a
  ///   std::shared_ptr<const schedule::Database> for this. The negotiation will
  ///   avoid creating any new conflicts with schedule participants that are not
  ///   part of the negotiation.
  CentralizedNegotiation(std::shared_ptr<const schedule::Viewer> viewer);

  /// Get the schedule viewer
  const std::shared_ptr<const schedule::Viewer>& viewer() const;

  /// Set the schedule viewer
  CentralizedNegotiation& viewer(std::shared_ptr<const schedule::Viewer> v);

  /// Require the negotiation to consider all combinations so that it finds the
  /// (near-)optimal solution. Off by default.
  CentralizedNegotiation& optimal(bool on = true);

  /// Toggle on/off whether to log the progress of the negotiation and save it
  /// in the Result. Off by default.
  CentralizedNegotiation& log(bool on = true);

  /// Toggle on/off whether to print the progress of the negotiation while it is
  /// running. Off by default.
  CentralizedNegotiation& print(bool on = true);

  /// Solve a centralized negotiation for the given agents.
  Result solve(const std::vector<Agent>& agents) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__AGV__CENTRALIZEDNEGOTIATION_HPP
