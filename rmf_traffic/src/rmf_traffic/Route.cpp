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

#include "internal_Route.hpp"

#include <rmf_utils/Modular.hpp>

namespace rmf_traffic {

//==============================================================================
bool Dependency::operator==(const Dependency& other) const
{
  return on_participant == other.on_participant
      && on_plan == other.on_plan
      && on_route == other.on_route
      && on_checkpoint == other.on_checkpoint;
}

//==============================================================================
class DependsOnPlan::Implementation
{
public:

  std::optional<PlanId> plan;
  DependsOnRoute routes;

};

//==============================================================================
DependsOnPlan::DependsOnPlan()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
DependsOnPlan::DependsOnPlan(PlanId plan, DependsOnRoute routes)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{plan, std::move(routes)}))
{
  // Do nothing
}

//==============================================================================
DependsOnPlan& DependsOnPlan::plan(std::optional<PlanId> plan)
{
  _pimpl->plan = plan;
  return *this;
}

//==============================================================================
std::optional<PlanId> DependsOnPlan::plan() const
{
  return _pimpl->plan;
}

//==============================================================================
DependsOnPlan& DependsOnPlan::routes(DependsOnRoute routes)
{
  _pimpl->routes = std::move(routes);
  return *this;
}

//==============================================================================
DependsOnRoute& DependsOnPlan::routes()
{
  return _pimpl->routes;
}

//==============================================================================
const DependsOnRoute& DependsOnPlan::routes() const
{
  return _pimpl->routes;
}

//==============================================================================
DependsOnPlan& DependsOnPlan::add_dependency(
  const CheckpointId dependent_checkpoint,
  const Dependency dep)
{
  const auto insertion = _pimpl->routes[dep.on_route]
    .insert({dep.on_checkpoint, dependent_checkpoint});

  if (!insertion.second)
  {
    // If the dependent checkpoint already has a dependency on this route, then
    // we should check if the new other_checkpoint is larger than the one that
    // already there.
    auto& prior_checkpoint = insertion.first->second;
    if (dependent_checkpoint < prior_checkpoint)
    {
      prior_checkpoint = dependent_checkpoint;
    }
  }

  return *this;
}

//==============================================================================
Route::Route(
  std::string map,
  Trajectory trajectory)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(map),
        std::move(trajectory),
        {},
        {}
      }))
{
  // Do nothing
}

//==============================================================================
Route& Route::map(std::string value)
{
  _pimpl->map = std::move(value);
  return *this;
}

//==============================================================================
const std::string& Route::map() const
{
  return _pimpl->map;
}

//==============================================================================
Route& Route::trajectory(Trajectory value)
{
  _pimpl->trajectory = std::move(value);
  return *this;
}

//==============================================================================
Trajectory& Route::trajectory()
{
  return _pimpl->trajectory;
}

//==============================================================================
const Trajectory& Route::trajectory() const
{
  return _pimpl->trajectory;
}

//==============================================================================
Route& Route::checkpoints(std::set<uint64_t> value)
{
  _pimpl->checkpoints = std::move(value);
  return *this;
}

//==============================================================================
std::set<uint64_t>& Route::checkpoints()
{
  return _pimpl->checkpoints;
}

//==============================================================================
const std::set<uint64_t>& Route::checkpoints() const
{
  return _pimpl->checkpoints;
}

//==============================================================================
Route& Route::dependencies(DependsOnParticipant value)
{
  _pimpl->dependencies = std::move(value);
  return *this;
}

//==============================================================================
DependsOnParticipant& Route::dependencies()
{
  return _pimpl->dependencies;
}

//==============================================================================
const DependsOnParticipant& Route::dependencies() const
{
  return _pimpl->dependencies;
}

//==============================================================================
Route& Route::add_dependency(
  const CheckpointId dependent_checkpoint,
  const Dependency dep)
{
  auto& depends_on_plan = _pimpl->dependencies[dep.on_participant];
  if (depends_on_plan.plan().has_value())
  {
    // If the new dependency is for an earlier plan than the current one, we
    // will ignore it.
    // TODO(MXG): Should we consider throwing an exception instead?
    if (rmf_utils::modular(dep.on_plan).less_than(*depends_on_plan.plan()))
    {
      return *this;
    }
    else if (dep.on_plan != *depends_on_plan.plan())
    {
      // A newer plan exists for this other participant, so we will clear out
      // the old list of dependencies.
      depends_on_plan.routes().clear();
    }
  }

  depends_on_plan.plan(dep.on_plan);
  depends_on_plan.add_dependency(
    dependent_checkpoint, {dep.on_route, dep.on_checkpoint});
  return *this;
}

//==============================================================================
bool Route::should_ignore(
  const ParticipantId participant,
  const PlanId plan) const
{
  const auto p_it = _pimpl->dependencies.find(participant);
  if (p_it == _pimpl->dependencies.end())
    return false;

  if (!p_it->second.plan().has_value())
    return false;

  return rmf_utils::modular(plan).less_than(*p_it->second.plan());
}

//==============================================================================
const DependsOnCheckpoint* Route::check_dependencies(
  const ParticipantId on_participant,
  const PlanId on_plan,
  const RouteId on_route) const
{
  const auto p_it = _pimpl->dependencies.find(on_participant);
  if (p_it == _pimpl->dependencies.end())
  {
    return nullptr;
  }

  const auto& plan_deps = p_it->second;
  const auto plan_deps_id = plan_deps.plan();
  if (!plan_deps_id.has_value())
  {
    return nullptr;
  }

  if (*plan_deps_id != on_plan)
  {
    return nullptr;
  }

  const auto& routes = plan_deps.routes();
  const auto r_it = routes.find(on_route);
  if (r_it == routes.end())
  {
    return nullptr;
  }

  return &r_it->second;
}

} // namespace rmf_traffic
