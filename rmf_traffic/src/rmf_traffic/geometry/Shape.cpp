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

#include <rmf_traffic/geometry/Shape.hpp>

#include "ShapeInternal.hpp"

namespace rmf_traffic {
namespace geometry {

//==============================================================================
Shape::Internal* Shape::_get_internal()
{
  return _internal.get();
}

//==============================================================================
const Shape::Internal* Shape::_get_internal() const
{
  return _internal.get();
}

//==============================================================================
Shape::Shape(std::unique_ptr<Internal> internal)
: _internal(std::move(internal))
{
  // Do nothing
}

//==============================================================================
Shape::~Shape()
{
  // Do nothing
}

//==============================================================================
const Shape& FinalShape::source() const
{
  return *_pimpl->_shape;
}

//==============================================================================
double FinalShape::get_characteristic_length() const
{
  return _pimpl->_characteristic_length;
}

//==============================================================================
FinalShape::FinalShape()
{
  // Do nothing. The _pimpl will be constructed by
  // Implementation::make_final_shape
}

//==============================================================================
bool FinalShape::operator==(const FinalShape& other) const
{
  return _pimpl->_compare_equality(*(other._pimpl->_shape));
}

//==============================================================================
bool FinalShape::operator!=(const FinalShape& other) const
{
  return !(*this == other);
}

} // namespace geometry
} // namespace rmf_traffic
