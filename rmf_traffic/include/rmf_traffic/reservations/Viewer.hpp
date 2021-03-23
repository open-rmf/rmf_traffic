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

#ifndef RMF_TRAFFIC__RESERVATION__VIEWER_HPP
#define RMF_TRAFFIC__RESERVATION__VIEWER_HPP

#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/reservations/ReservationRequest.hpp>
#include <rmf_traffic/reservations/Query.hpp>
#include <rmf_traffic/detail/bidirectional_iterator.hpp>
#include <vector>

namespace rmf_traffic {
namespace reservations {
class Viewer
{
public:
  template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::bidirectional_iterator<E, I, F>;
  class View
  {
  public:
    class IterImpl;
    using const_iterator = base_iterator<const Reservation, IterImpl, View>;
    using iterator = const_iterator;

    /// Returns an iterator to the first element of the View
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// View.
    const_iterator end() const;

    /// Returns the number of elements in this View.
    std::size_t size() const;

    class Implementation;

    private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  virtual View query(Query query) = 0;
};
}
}

#endif