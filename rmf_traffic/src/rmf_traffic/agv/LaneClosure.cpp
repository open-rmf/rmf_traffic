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
#include <unordered_map>

namespace rmf_traffic {
namespace agv {

//==============================================================================
class LaneClosure::Implementation
{
public:

  bool contains(std::size_t value) const
  {
    assert(field_size == 64);
    const std::size_t key = _get_key(value);
    const auto bucket_it = _bitfields.find(key);
    if (bucket_it == _bitfields.end())
      return false;

    const std::size_t bit = _get_bit(value);
    const std::size_t bitfield = bucket_it->second;
    return static_cast<bool>(bitfield & bit);
  }

  void insert(std::size_t value)
  {
    const std::size_t key = _get_key(value);
    const std::size_t bit = _get_bit(value);

    const auto insertion = _bitfields.insert({key, bit});
    if (insertion.second)
    {
      // The bitfield did not exist before, so now it has been inserted with
      // the desired bit. We will need to recalculate the hash. After that, we
      // can return.
      _recalculate_hash();
      return;
    }

    std::size_t& bitfield = insertion.first->second;
    if (static_cast<bool>(bitfield & bit))
    {
      // The map already contained this value, so we do not need to insert it
      // or recalculate the hash.
      return;
    }

    bitfield |= bit;
    _recalculate_hash();
  }

  void erase(std::size_t value)
  {
    const std::size_t key = _get_key(value);
    const auto bucket_it = _bitfields.find(key);
    if (bucket_it == _bitfields.end())
    {
      // There was no bitfield for this value, so it is not in the map anyway.
      // We can simply return here.
      return;
    }

    const std::size_t bit = _get_bit(value);
    std::size_t& bitfield = bucket_it->second;
    if (!static_cast<bool>(bitfield & bit))
    {
      // This bit was not active in the bitfield, so we do not need to do
      // anything.
      return;
    }

    bitfield &= ~bit;
    _recalculate_hash();
  }

  std::size_t hash() const
  {
    return _hash;
  }

  bool operator==(const Implementation& other) const
  {
    return _bitfields == other._bitfields;
  }

private:

  std::unordered_map<std::size_t, std::size_t> _bitfields;
  std::size_t _hash = 0;

  static constexpr std::size_t field_size = sizeof(std::size_t)*8;

  void _recalculate_hash()
  {
    _hash = 0;
    for (const auto& [_, bitfield] : _bitfields)
      _hash |= bitfield;
  }

  std::size_t _get_key(const std::size_t value) const
  {
    return value/field_size;
  }

  std::size_t _get_bit(const std::size_t value) const
  {
    const std::size_t shift = value % field_size;
    return std::size_t(1) << shift;
  }
};

//==============================================================================
LaneClosure::LaneClosure()
: _pimpl(rmf_utils::make_impl<Implementation>())
{
  // Do nothing
}

//==============================================================================
bool LaneClosure::is_open(const std::size_t lane) const
{
  return !is_closed(lane);
}

//==============================================================================
bool LaneClosure::is_closed(const std::size_t lane) const
{
  return _pimpl->contains(lane);
}

//==============================================================================
LaneClosure& LaneClosure::open(const std::size_t lane)
{
  _pimpl->erase(lane);
  return *this;
}

//==============================================================================
LaneClosure& LaneClosure::close(std::size_t lane)
{
  _pimpl->insert(lane);
  return *this;
}

//==============================================================================
std::size_t LaneClosure::hash() const
{
  return _pimpl->hash();
}

//==============================================================================
bool LaneClosure::operator==(const LaneClosure& other) const
{
  return *_pimpl == *other._pimpl;
}

} // namespace agv
} // namespace rmf_traffic
