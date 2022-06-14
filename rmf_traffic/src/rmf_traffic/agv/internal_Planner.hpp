/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#ifndef SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNER_HPP
#define SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNER_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include "internal_planning.hpp"
#include "planning/Tree.hpp"

namespace rmf_traffic {
namespace agv {

//==============================================================================
class Plan::Waypoint::Implementation
{
public:

  Eigen::Vector3d position;

  Time time;

  std::optional<std::size_t> graph_index;

  std::vector<std::size_t> approach_lanes;

  std::vector<Progress> progress;

  Checkpoints arrival;

  Graph::Lane::EventPtr event;

  Dependencies dependencies = {};

  template<typename... Args>
  static Waypoint make(Args&& ... args)
  {
    Waypoint wp;
    wp._pimpl = rmf_utils::make_impl<Implementation>(
      Implementation{std::forward<Args>(args)...});

    return wp;
  }

//  static Waypoint copy(const Implementation& impl)
//  {
//    Waypoint wp;
//    wp._pimpl = rmf_utils::make_impl<Implementation>(std::move(impl));
//    return wp;
//  }

  static void add_dependency(Waypoint& waypoint, const Dependency dep)
  {
    waypoint._pimpl->dependencies.push_back(dep);
  }
};

//==============================================================================
class Planner::Result::Implementation
{
public:

  planning::InterfacePtr interface;
  planning::State state;
  std::optional<Plan> plan;

  static Result generate(
    planning::InterfacePtr interface,
    const std::vector<Planner::Start>& starts,
    Planner::Goal goal,
    Planner::Options options);

  static Result setup(
    planning::InterfacePtr interface,
    const std::vector<Planner::Start>& starts,
    Planner::Goal goal,
    Planner::Options options);

  static const Implementation& get(const Result& r);

};

//==============================================================================
class Planner::QuickestPath::Implementation
{
public:
  planning::ConstForestSolutionPtr solution;
  double cost_offset;

  static void choose_better(
    std::optional<Implementation>& left,
    const Implementation& right);

  static std::optional<QuickestPath> promote(
    std::optional<Implementation> value);
};

} // namespace agv
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__AGV__INTERNAL_PLANNER_HPP
