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
bool Profile::operator==(const Profile & rhs) const
{
  // TODO(geoff): Something in here segfaults
  if (!_pimpl && !rhs._pimpl)
  {
    return true;
  }
  else if (!_pimpl || !rhs._pimpl)
  {
    return false;
  }

  bool r1 = false;
  if (_pimpl->footprint && rhs._pimpl->footprint)
  {
    // Both pointers are valid so check what they point to for equality
    r1 = *_pimpl->footprint == *rhs._pimpl->footprint;
  }
  else if (_pimpl->footprint || rhs._pimpl->footprint)
  {
    // Only one pointer is valid, so they can't be equal
    r1 = false;
  }
  else
  {
    // Both pointers are invalid, so equal
    r1 = true;
  }

  bool r2 = false;
  if (_pimpl->vicinity && rhs._pimpl->vicinity)
  {
    // Both pointers are valid so check what they point to for equality
    r2 = *_pimpl->footprint == *rhs._pimpl->footprint;
  }
  else if (_pimpl->vicinity || rhs._pimpl->vicinity)
  {
    // Only one pointer is valid, so they can't be equal
    r2 = false;
  }
  else
  {
    // Both pointers are invalid, so equal
    r2 = true;
  }

  return r1 && r2;
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
