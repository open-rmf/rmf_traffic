/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef RMF_TRAFFIC__AGV__LANECLOSURE_HPP
#define RMF_TRAFFIC__AGV__LANECLOSURE_HPP

#include <utility>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace agv {

//==============================================================================
/// This class describes the closure status of lanes in a Graph, i.e. whether a
/// lane is open or closed. Open lanes can be used by the planner to reach a
/// goal. The planner will not expand down a lane that is closed.
class LaneClosure
{
public:

  /// Default constructor.
  ///
  /// By default, all lanes are open.
  LaneClosure();

  /// Check whether the lane corresponding to the given index is open.
  ///
  /// \param[in] lane
  ///   The index for the lane of interest
  bool is_open(std::size_t lane) const;

  /// Check whether the lane corresponding to the given index is closed.
  ///
  /// \param[in] lane
  ///   The index for the lane of interest
  bool is_closed(std::size_t lane) const;

  /// Set the lane corresponding to the given index to be open.
  ///
  /// \param[in] lane
  ///   The index for the opening lane
  LaneClosure& open(std::size_t lane);

  /// Set the lane corresponding to the given index to be closed.
  ///
  /// \param[in] lane
  ///   The index for the closing lane
  LaneClosure& close(std::size_t lane);

  /// Get an integer that uniquely describes the overall closure status of the
  /// graph lanes.
  std::size_t hash() const;

  /// Equality comparison operator
  bool operator==(const LaneClosure& other) const;

  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace agv
} // namespace rmf_traffic

namespace std {
//==============================================================================
template<>
struct hash<rmf_traffic::agv::LaneClosure>
{
  std::size_t operator()(
    const rmf_traffic::agv::LaneClosure& closure) const noexcept
  {
    return closure.hash();
  }
};
} // namespace std

#endif // RMF_TRAFFIC__AGV__LANECLOSURE_HPP

