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

#include <rmf_traffic/agv/LaneClosure.hpp>
#include <vector>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class LaneClosure::Implementation
{
public:

  std::vector<std::size_t> bitfield;

  bool contains(std::size_t value) const
  {
    const std::size_t bucket = value/sizeof(std::size_t);
    if (bitfield.size() <= bucket)
      return false;


    const std::size_t compare = 1 << lane;
    return static_cast<bool>(_pimpl->hash & compare);
  }

};

//==============================================================================
bool LaneClosure::is_open(const std::size_t lane) const
{
  return !_pimpl->contains(lane);
}

//==============================================================================
bool LaneClosure::is_closed(const std::size_t lane) const
{
  return _pimpl->contains(lane);
}

//==============================================================================
LaneClosure& LaneClosure::open(const std::size_t lane)
{
  const std::size_t bit = 1 << lane;
  _pimpl->hash = (_pimpl->hash & ~bit);
  return *this;
}

//==============================================================================
LaneClosure& LaneClosure::close(std::size_t lane)
{
  const std::size_t bit = 1 << lane;
  _pimpl->hash = (_pimpl->hash | bit);
  return *this;
}

//==============================================================================
std::size_t LaneClosure::hash() const
{
  return _pimpl->hash;
}

//==============================================================================
bool LaneClosure::operator==(const LaneClosure& other) const
{

}

} // namespace agv
} // namespace rmf_traffic
