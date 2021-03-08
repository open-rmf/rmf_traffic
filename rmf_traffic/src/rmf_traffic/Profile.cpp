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
{
  geometry::ConstFinalConvexShapeGroup footprint_grp;
  footprint_grp.emplace_back(footprint);
  geometry::ConstFinalConvexShapeGroup vicinity_grp;

  if (vicinity)
    vicinity_grp.emplace_back(vicinity);
  *this = Profile(footprint_grp, vicinity_grp);
}

Profile::Profile(
    geometry::ConstFinalConvexShapeGroup footprint,
    geometry::ConstFinalConvexShapeGroup vicinity)
: _pimpl(rmf_utils::make_impl<Implementation>(
      Implementation{
        std::move(footprint),
        std::move(vicinity)
      }))
{
  _pimpl->footprint_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->footprint);
  if (_pimpl->vicinity.size())
    _pimpl->vicinity_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->vicinity);
  else
    _pimpl->vicinity_characteristic_length = _pimpl->footprint_characteristic_length;
}

//==============================================================================
Profile& Profile::footprint(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->footprint.clear();
  _pimpl->footprint.emplace_back(std::move(shape));
  _pimpl->footprint_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->footprint);
  if (_pimpl->vicinity.empty())
    _pimpl->vicinity_characteristic_length = _pimpl->footprint_characteristic_length;
  return *this;
}

//==============================================================================
Profile& Profile::footprint(geometry::ConstFinalConvexShapeGroup shape)
{
  _pimpl->footprint = std::move(shape);
  _pimpl->footprint_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->footprint);
  if (_pimpl->vicinity.empty())
    _pimpl->vicinity_characteristic_length = _pimpl->footprint_characteristic_length;
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapeGroup& Profile::footprint() const
{
  return _pimpl->footprint;
}

//==============================================================================
double Profile::get_footprint_characteristic_length() const
{
  return _pimpl->footprint_characteristic_length;
}

//==============================================================================
Profile& Profile::vicinity(geometry::ConstFinalConvexShapePtr shape)
{
  _pimpl->vicinity.clear();
  _pimpl->vicinity.emplace_back(std::move(shape));
  _pimpl->vicinity_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->vicinity);
  return *this;
}

//==============================================================================
Profile& Profile::vicinity(geometry::ConstFinalConvexShapeGroup shape)
{
  _pimpl->vicinity = std::move(shape);
  _pimpl->vicinity_characteristic_length = _pimpl->compute_shapegroup_characteristic_length(_pimpl->vicinity);
  return *this;
}

//==============================================================================
const geometry::ConstFinalConvexShapeGroup& Profile::vicinity() const
{
  if (_pimpl->vicinity.size())
    return _pimpl->vicinity;

  return _pimpl->footprint;
}

//==============================================================================
double Profile::get_vicinity_characteristic_length() const
{
  return _pimpl->vicinity_characteristic_length;
}

} // namespace rmf_traffic
