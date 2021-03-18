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


#ifndef RMF_TRAFFIC__GEOMETRY__BOX_HPP
#define RMF_TRAFFIC__GEOMETRY__BOX_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

namespace rmf_traffic {
namespace geometry {

// TODO(MXG): This header has been moved out of the public API because our
// collision detection does not properly support it yet. This should be moved
// back to the public API once the support is available.

//==============================================================================
/// \brief This class represent a box shape which can be added into a Zone or
/// Trajectory.
class Box : public ConvexShape
{
public:

  /// \brief Create a box with the indicated dimensions.
  ///
  /// The origin of the box will be in its center, so the box will extend:
  /// [-x_length/2.0, x_length/2.0] along its x axis and
  /// [-y_length/2.0, y_length/2.0] along its y axis.
  Box(double x_length, double y_length);

  // The typical copy constructor/assignment operator
  Box(const Box& other);
  Box& operator=(const Box& other);

  /// \brief Set the length of the box along its x axis
  void set_x_length(double x_length);

  /// \brief Set the length of the box along its y axis
  void set_y_length(double y_length);

  /// \brief Get the length of the box along its x axis
  double get_x_length() const;

  /// \brief Get the length of the box along its y axis
  double get_y_length() const;

  // Docuemntation inherited
  FinalShape finalize() const final;

  // Documentation inherited
  FinalConvexShape finalize_convex() const final;

};

//==============================================================================
/// Equality operator for Box objects.
///
/// \param[in] lhs
///   A const reference to the left-hand-side of the comparison.
///
/// \param[in] rhs
///   A const reference to the right-hand-side of the comparison.
bool operator==(
  const Box& lhs,
  const Box& rhs);

//==============================================================================
/// Non-equality operator for Box objects.
///
/// \param[in] lhs
///   A const reference to the left-hand-side of the comparison.
///
/// \param[in] rhs
///   A const reference to the right-hand-side of the comparison.
bool operator!=(
  const Box& lhs,
  const Box& rhs);

} // namespace geometry
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__GEOMETRY__BOX_HPP
