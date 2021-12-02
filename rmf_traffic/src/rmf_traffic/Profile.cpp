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

#include "ProfileInternal.hpp"

namespace rmf_traffic {

//==============================================================================
Profile::Profile(
  geometry::ConstFinalConvexShapePtr footprint,
  geometry::ConstFinalConvexShapePtr vicinity)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(footprint),
        std::move(vicinity)
      }))
{
  // Do nothing
}

//==============================================================================
bool Profile::operator==(const Profile& rhs) const
{
  // TODO(MXG): We should write tests for the shape and profile comparisons.
  if (!_pimpl && !rhs._pimpl)
  {
    return true;
  }
  else if (!_pimpl || !rhs._pimpl)
  {
    return false;
  }

  const auto& self_footprint = footprint();
  const auto& other_footprint = rhs.footprint();
  if (self_footprint && other_footprint)
  {
    // Both pointers are valid so check what they point to for equality
    if (*self_footprint != *other_footprint)
      return false;
  }
  else if (self_footprint || other_footprint)
  {
    // Only one pointer is valid, so they can't be equal
    return false;
  }

  // Using the getter functions for vicinity is important because the Profile
  // class has a behavior where it will return the footprint as the vicinity
  // if a vicinity was never specified.
  const auto& self_vicinity = vicinity();
  const auto& other_vicinity = rhs.vicinity();
  if (self_vicinity && other_vicinity)
  {
    // Both pointers are valid so check what they point to for equality
    if (*self_vicinity != *other_vicinity)
      return false;
  }
  else if (_pimpl->vicinity || rhs._pimpl->vicinity)
  {
    // Only one pointer is valid, so they can't be equal
    return false;
  }

  return true;
}

//==============================================================================
Profile& Profile::footprint(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->footprint = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& Profile::footprint() const
{
  return _pimpl->footprint;
}

//==============================================================================
Profile& Profile::vicinity(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->vicinity = std::move(shape);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapePtr& Profile::vicinity() const
{
  if (_pimpl->vicinity)
    return _pimpl->vicinity;

  return _pimpl->footprint;
}

} // namespace rmf_traffic
