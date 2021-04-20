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


#ifndef RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP
#define RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP

#include <rmf_traffic/geometry/ConvexShape.hpp>

namespace rmf_traffic {
namespace geometry {

//==============================================================================
/// \brief This class represent a circle shape which can be added into a Zone or
/// Trajectory.
class Circle : public ConvexShape
{
public:

  Circle(double radius);

  // The typical copy constructor/assignment operator
  Circle(const Circle& other);
  Circle& operator=(const Circle& other);

  void set_radius(double r);

  double get_radius() const;

  // Docuemntation inherited
  FinalShape finalize() const final;

  // Docuemntation inherited
  FinalConvexShape finalize_convex() const final;

};

//==============================================================================
/// Equality operator for Circle objects.
///
/// \param[in] lhs
///   A const reference to the left-hand-side of the comparison.
///
/// \param[in] rhs
///   A const reference to the right-hand-side of the comparison.
bool operator==(
  const Circle& lhs,
  const Circle& rhs);

//==============================================================================
/// Non-equality operator for Circle objects.
///
/// \param[in] lhs
///   A const reference to the left-hand-side of the comparison.
///
/// \param[in] rhs
///   A const reference to the right-hand-side of the comparison.
bool operator!=(
  const Circle& lhs,
  const Circle& rhs);

} // namespace geometry
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__GEOMETRY__CIRCLE_HPP
