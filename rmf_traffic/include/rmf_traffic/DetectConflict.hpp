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

#ifndef RMF_TRAFFIC__DETECTCONFLICT_HPP
#define RMF_TRAFFIC__DETECTCONFLICT_HPP

#include <rmf_traffic/Trajectory.hpp>
#include <rmf_traffic/Route.hpp>
#include <rmf_traffic/Profile.hpp>
#include <exception>

namespace rmf_traffic {

namespace {


} // anonymous namespace

//==============================================================================
class invalid_trajectory_error : public std::exception
{
public:

  const char* what() const noexcept override;

  class Implementation;
private:
  invalid_trajectory_error();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

//==============================================================================
class DetectConflict
{
public:

  enum class Interpolate : uint16_t
  {
    CubicSpline
  };

  struct Conflict
  {
    Trajectory::const_iterator a_it;
    Trajectory::const_iterator b_it;
    Time time;
  };

  /// Checks if there are any conflicts between the two trajectories.
  ///
  /// \param[in] profile_a
  ///   The profile of agent A
  ///
  /// \param[in] trajectory_a
  ///   The trajectory of agent A
  ///
  /// \param[in] dependencies_of_a_on_b
  ///   The dependencies that agent A has on the given trajectory of agent B
  ///
  /// \param[in] profile_b
  ///   The profile of agent B
  ///
  /// \param[in] trajectory_b
  ///   The trajectory of agent B
  ///
  /// \param[in] dependencies_of_b_on_a
  ///   The dependencies that agent B has on the given trajectory of agent A
  ///
  /// \return true if a conflict exists between the trajectories, false
  /// otherwise.
  static std::optional<Conflict> between(
    const Profile& profile_a,
    const Trajectory& trajectory_a,
    const DependsOnCheckpoint* dependencies_of_a_on_b,
    const Profile& profile_b,
    const Trajectory& trajectory_b,
    const DependsOnCheckpoint* dependencies_of_b_on_a,
    Interpolate interpolation = Interpolate::CubicSpline);

  class Implementation;
};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__DETECTCONFLICT_HPP
