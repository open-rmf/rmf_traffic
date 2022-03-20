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

#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/DetectConflict.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class ScheduleRouteValidator::Implementation
{
public:

  std::shared_ptr<const schedule::Viewer> shared_viewer;
  const schedule::Viewer* viewer;
  schedule::ParticipantId participant;
  Profile profile;

};

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  const schedule::Viewer& viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        nullptr,
        &viewer,
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
ScheduleRouteValidator::ScheduleRouteValidator(
  std::shared_ptr<const schedule::Viewer> viewer,
  schedule::ParticipantId participant_id,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        viewer,
        viewer.get(),
        participant_id,
        std::move(profile)
      }))
{
  // Do nothing
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::schedule_viewer(
  const schedule::Viewer& viewer)
{
  _pimpl->viewer = &viewer;
  return *this;
}

//==============================================================================
const schedule::Viewer& ScheduleRouteValidator::schedule_viewer() const
{
  return *_pimpl->viewer;
}

//==============================================================================
ScheduleRouteValidator& ScheduleRouteValidator::participant(
  const schedule::ParticipantId p)
{
  _pimpl->participant = p;
  return *this;
}

//==============================================================================
schedule::ParticipantId ScheduleRouteValidator::participant() const
{
  return _pimpl->participant;
}

//==============================================================================
std::optional<RouteValidator::Conflict>
ScheduleRouteValidator::find_conflict(const Route& route) const
{
  // TODO(MXG): Should we use a mutable Spacetime instance to avoid the
  // allocation here?
  schedule::Query::Spacetime spacetime;
  spacetime.query_timespan()
  .all_maps(false)
  .add_map(route.map())
  .set_lower_time_bound(*route.trajectory().start_time())
  .set_upper_time_bound(*route.trajectory().finish_time());

  const auto view = _pimpl->viewer->query(
    spacetime, schedule::Query::Participants::make_all());

  for (const auto& v : view)
  {
    if (v.participant == _pimpl->participant)
      continue;

    if (const auto conflict = rmf_traffic::DetectConflict::between(
        _pimpl->profile,
        route.trajectory(),
        route.check_dependencies(v.participant, v.plan_id, v.route_id),
        v.description.profile(),
        v.route->trajectory(),
        nullptr))
    {
      return Conflict{
        Dependency{
          v.participant,
          v.plan_id,
          v.route_id,
          v.route->trajectory().index_after(conflict->time)
        },
        conflict->time,
        v.route
      };
    }
  }

  return std::nullopt;
}

//==============================================================================
std::unique_ptr<RouteValidator> ScheduleRouteValidator::clone() const
{
  return std::make_unique<ScheduleRouteValidator>(*this);
}

//==============================================================================
class NegotiatingRouteValidator::Generator::Implementation
{
public:
  struct Data
  {
    std::unordered_set<schedule::ParticipantId> stakeholders;
    schedule::Negotiation::Table::ViewerPtr viewer;
    Profile profile;
    bool ignore_unresponsive;
    bool ignore_bystanders;
  };

  std::shared_ptr<Data> data;
  std::vector<schedule::ParticipantId> alternative_sets;

  static std::unordered_set<schedule::ParticipantId> get_stakeholders(
    const schedule::Negotiation::Table::ViewerPtr& viewer)
  {
    std::unordered_set<schedule::ParticipantId> stakeholders;
    for (const auto& p : viewer->sequence())
      stakeholders.insert(p.participant);

    return stakeholders;
  }

  Implementation(
    schedule::Negotiation::Table::ViewerPtr viewer,
    Profile profile)
  : data(std::make_shared<Data>(
        Data{
          get_stakeholders(viewer),
          std::move(viewer),
          std::move(profile),
          false,
          false
        }))
  {
    const auto& alternatives = data->viewer->alternatives();
    alternative_sets.reserve(alternatives.size());
    for (const auto& r : alternatives)
      alternative_sets.push_back(r.first);
  }
};

//==============================================================================
class NegotiatingRouteValidator::Implementation
{
public:

  std::shared_ptr<const Generator::Implementation::Data> data;
  schedule::Negotiation::VersionedKeySequence rollouts;
  std::optional<schedule::ParticipantId> masked = std::nullopt;

  static NegotiatingRouteValidator make(
    std::shared_ptr<const Generator::Implementation::Data> data,
    schedule::Negotiation::VersionedKeySequence rollouts)
  {
    NegotiatingRouteValidator output;
    output._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(data),
        std::move(rollouts)
      });

    return output;
  }

  static rmf_utils::clone_ptr<NegotiatingRouteValidator> make_ptr(
    std::shared_ptr<const Generator::Implementation::Data> data,
    schedule::Negotiation::VersionedKeySequence rollout)
  {
    return rmf_utils::make_clone<NegotiatingRouteValidator>(
      make(std::move(data), std::move(rollout)));
  }
};

//==============================================================================
NegotiatingRouteValidator::Generator::Generator(
  schedule::Negotiation::Table::ViewerPtr viewer,
  Profile profile)
: _pimpl(rmf_utils::make_impl<Implementation>(
      std::move(viewer), std::move(profile)))
{
  // Do nothing
}

//==============================================================================
NegotiatingRouteValidator::Generator::Generator(
  schedule::Negotiation::Table::ViewerPtr viewer)
: _pimpl(rmf_utils::make_impl<Implementation>(
      viewer,
      viewer->get_description(viewer->participant_id())->profile()))
{
  // Do nothing
}

//==============================================================================
auto NegotiatingRouteValidator::Generator::ignore_unresponsive(const bool val)
-> Generator&
{
  _pimpl->data->ignore_unresponsive = val;
  return *this;
}

//==============================================================================
auto NegotiatingRouteValidator::Generator::ignore_bystanders(const bool val)
-> Generator&
{
  _pimpl->data->ignore_bystanders = val;
  return *this;
}

//==============================================================================
NegotiatingRouteValidator NegotiatingRouteValidator::Generator::begin() const
{
  schedule::Negotiation::VersionedKeySequence rollouts;
  for (const auto& r : _pimpl->data->viewer->alternatives())
    rollouts.push_back({r.first, 0});

  return NegotiatingRouteValidator::Implementation::make(
    _pimpl->data, std::move(rollouts));
}

//==============================================================================
std::vector<rmf_utils::clone_ptr<NegotiatingRouteValidator>>
NegotiatingRouteValidator::Generator::all() const
{
  const std::size_t N_alts = _pimpl->data->viewer->alternatives().size();
  if (0 == N_alts)
    return {rmf_utils::make_clone<NegotiatingRouteValidator>(begin())};

  std::vector<std::vector<schedule::Version>> version_queue;
  std::vector<schedule::Version> current_versions;
  current_versions.reserve(N_alts);
  std::vector<schedule::Version> end_versions;
  current_versions.reserve(N_alts);
  schedule::Negotiation::VersionedKeySequence keys;
  keys.reserve(N_alts);
  for (const auto& alts : _pimpl->data->viewer->alternatives())
  {
    current_versions.push_back(0);
    end_versions.push_back(alts.second->size());
    keys.push_back({alts.first, 0});
  }

  assert(current_versions.size() == N_alts);
  assert(end_versions.size() == N_alts);
  assert(keys.size() == N_alts);

  while (true) // A break statement provides the exit condition
  {
    for (std::size_t i = 0; i < N_alts-1; ++i)
    {
      if (current_versions[i] >= end_versions[i])
      {
        for (std::size_t j = 0; j <= i; ++j)
          current_versions[j] = 0;

        ++current_versions[i+1];
        continue;
      }
    }

    if (current_versions.back() >= end_versions.back())
      break;

    version_queue.push_back(current_versions);
    ++current_versions[0];
  }

  std::vector<rmf_utils::clone_ptr<NegotiatingRouteValidator>> validators;
  validators.reserve(version_queue.size());

  for (const auto& versions : version_queue)
  {
    for (std::size_t i = 0; i < N_alts; ++i)
      keys[i].version = versions[i];

    validators.emplace_back(
      NegotiatingRouteValidator::Implementation::make_ptr(
        _pimpl->data, keys));
  }

  return validators;
}

//==============================================================================
const std::vector<schedule::ParticipantId>&
NegotiatingRouteValidator::Generator::alternative_sets() const
{
  return _pimpl->alternative_sets;
}

//==============================================================================
std::size_t NegotiatingRouteValidator::Generator::alternative_count(
  schedule::ParticipantId participant) const
{
  return _pimpl->data->viewer->alternatives().at(participant)->size();
}

//==============================================================================
NegotiatingRouteValidator& NegotiatingRouteValidator::mask(
  schedule::ParticipantId id)
{
  _pimpl->masked = id;
  return *this;
}

//==============================================================================
NegotiatingRouteValidator& NegotiatingRouteValidator::remove_mask()
{
  _pimpl->masked = std::nullopt;
  return *this;
}

//==============================================================================
NegotiatingRouteValidator NegotiatingRouteValidator::next(
  schedule::ParticipantId id) const
{
  auto rollouts = _pimpl->rollouts;
  const auto it = std::find_if(
    rollouts.begin(), rollouts.end(), [&](
      const schedule::Negotiation::VersionedKey& key)
    {
      return key.participant == id;
    });

  if (it == rollouts.end())
  {
    std::string error = "[NegotiatingRouteValidator::next] Requested next "
      "alternative for " + std::to_string(id) + " but the only options are [";

    for (const auto r : rollouts)
      error += " " + std::to_string(r.participant);

    error += " ]";

    throw std::runtime_error(error);
  }

  it->version += 1;

  return _pimpl->make(_pimpl->data, std::move(rollouts));
}

//==============================================================================
const schedule::Negotiation::VersionedKeySequence&
NegotiatingRouteValidator::alternatives() const
{
  return _pimpl->rollouts;
}

//==============================================================================
NegotiatingRouteValidator::operator bool() const
{
  return !end();
}

//==============================================================================
bool NegotiatingRouteValidator::end() const
{
  for (const auto& r : _pimpl->rollouts)
  {
    const auto num_alternatives =
      _pimpl->data->viewer->alternatives().at(r.participant)->size();

    if (num_alternatives <= r.version)
      return true;
  }

  return false;
}

//==============================================================================
std::optional<RouteValidator::Conflict>
NegotiatingRouteValidator::find_conflict(const Route& route) const
{
  using namespace std::chrono_literals;

  // TODO(MXG): Consider if we can reduce the amount of heap allocation that's
  // needed here.
  schedule::Query::Spacetime spacetime;
  spacetime.query_timespan()
  .all_maps(false)
  .add_map(route.map())
  .set_lower_time_bound(*route.trajectory().start_time())
  .set_upper_time_bound(*route.trajectory().finish_time());

  const auto skip_unresponsive =
    [may_skip = _pimpl->data->ignore_unresponsive](
    const schedule::ParticipantDescription& description) -> bool
    {
      if (!may_skip)
        return false;

      return description.responsiveness()
        == schedule::ParticipantDescription::Rx::Unresponsive;
    };

  const auto skip_bystander =
    [data = _pimpl->data](const schedule::ParticipantId id) -> bool
    {
      if (!data->ignore_bystanders)
        return false;

      return data->stakeholders.find(id) == data->stakeholders.end();
    };

  const auto skip_participant =
    [skip_unresponsive, skip_bystander](
    const schedule::ParticipantId id,
    const schedule::ParticipantDescription& desc) -> bool
    {
      if (skip_unresponsive(desc))
        return true;

      if (skip_bystander(id))
        return true;

      return false;
    };

  const auto view = _pimpl->data->viewer->query(spacetime, _pimpl->rollouts);
  for (const auto& v : view)
  {
    if (_pimpl->masked && (*_pimpl->masked == v.participant))
      continue;

    if (skip_participant(v.participant, v.description))
      continue;

    // NOTE(MXG): There is no need to check the map, because the query will
    // filter out all itineraries that are not on this map.
    if (const auto conflict = rmf_traffic::DetectConflict::between(
        _pimpl->data->profile,
        route.trajectory(),
        route.check_dependencies(v.participant, v.plan_id, v.route_id),
        v.description.profile(),
        v.route->trajectory(),
        nullptr))
    {
      return Conflict{
        Dependency{
          v.participant,
          v.plan_id,
          v.route_id,
          v.route->trajectory().index_after(conflict->time)
        },
        conflict->time,
        v.route
      };
    }
  }

  {
    const auto initial_endpoints = _pimpl->data->viewer->initial_endpoints(
      _pimpl->rollouts);

    const auto& initial_wp = route.trajectory().front();
    for (const auto& other : initial_endpoints)
    {
      const auto& ep = other.second;
      if (skip_participant(ep.participant(), ep.description()))
        continue;

      if (route.map() != ep.map())
        continue;

      const auto& other_wp = ep.waypoint();
      if (other_wp.time() <= initial_wp.time())
        continue;

      Trajectory other_start;
      other_start.insert(
        initial_wp.time() - 1s,
        other_wp.position(),
        Eigen::Vector3d::Zero());

      other_start.insert(
        other_wp.time(),
        other_wp.position(),
        Eigen::Vector3d::Zero());

      if (const auto conflict = DetectConflict::between(
          _pimpl->data->profile,
          route.trajectory(),
          route.check_dependencies(other.first, ep.plan_id(), ep.route_id()),
          ep.description().profile(),
          other_start,
          nullptr))
      {
        return Conflict{
          Dependency{
            other.first,
            ep.plan_id(),
            ep.route_id(),
            other_start.index_after(conflict->time)
          },
          conflict->time,
          std::make_shared<Route>(route.map(), std::move(other_start))
        };
      }
    }
  }

  {
    const auto final_endpoints = _pimpl->data->viewer->final_endpoints(
      _pimpl->rollouts);

    const auto& final_wp = route.trajectory().back();
    for (const auto& other : final_endpoints)
    {
      const auto& ep = other.second;
      if (skip_participant(ep.participant(), ep.description()))
        continue;

      if (route.map() != ep.map())
        continue;

      const auto& other_wp = ep.waypoint();
      if (final_wp.time() <= other_wp.time())
        continue;

      Trajectory other_finish;
      other_finish.insert(
        other_wp.time(),
        other_wp.position(),
        Eigen::Vector3d::Zero());

      other_finish.insert(
        final_wp.time() + 1s,
        other_wp.position(),
        Eigen::Vector3d::Zero());

      if (const auto conflict = DetectConflict::between(
          _pimpl->data->profile,
          route.trajectory(),
          route.check_dependencies(other.first, ep.plan_id(), ep.route_id()),
          ep.description().profile(),
          other_finish,
          nullptr))
      {
        return Conflict{
          Dependency{
            other.first,
            ep.plan_id(),
            ep.route_id(),
            other_finish.index_after(conflict->time)
          },
          conflict->time,
          std::make_shared<Route>(route.map(), std::move(other_finish))
        };
      }
    }
  }

  return std::nullopt;
}

//==============================================================================
std::unique_ptr<RouteValidator> NegotiatingRouteValidator::clone() const
{
  return std::make_unique<NegotiatingRouteValidator>(*this);
}

//==============================================================================
NegotiatingRouteValidator::NegotiatingRouteValidator()
{
  // Do nothing
}

} // namespace agv
} // namespace rmf_traffic
