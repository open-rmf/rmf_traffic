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

#ifndef RMF_TRAFFIC__RESERVATION__PLANNER_GENERIC_PLAN_HPP
#define RMF_TRAFFIC__RESERVATION__PLANNER_GENERIC_PLAN_HPP

#include <rmf_traffic/reservations/Reservation.hpp>
#include <rmf_traffic/detail/forward_iterator.hpp>

#include <unordered_map>

namespace rmf_traffic {

namespace reservations {

struct Plan
{
  struct IndividualSchedule
  {
    std::vector<Reservation> push_back_reservations;
    std::vector<Reservation> bring_forward_reservations;
    std::vector<Reservation> to_remove;
    std::vector<Reservation> to_add;
  };
  std::unordered_map<std::string, IndividualSchedule> plan;
  float cost;
};

//=============================================================================
// A generic planner. The planner is designed to return a list of 
// plans one after the other. This is so that if one plan is not possible
// or fails to
class GenericPlanner
{
public:
  template<typename E, typename I, typename F>
    using base_iterator = rmf_traffic::detail::forward_iterator<E, I, F>;
  class GenericPlanIterator
  {
    class IterImpl;
    using const_iterator = base_iterator<Plan, IterImpl, GenericPlanIterator>;
    using iterator = const_iterator;

    /// Returns an iterator to the first element of the View
    const_iterator begin() const;

    /// Returns an iterator to the element following the last element of the
    /// View.
    const_iterator end() const;

    /// Returns the number of elements in this View.
    std::size_t size() const;

    class Implementation;

    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  virtual GenericPlanIterator plans() = 0;

  virtual ~GenericPlanner() = 0;
};

}
}
#endif