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

#ifndef RMF_TRAFFIC__RESERVATION_REQUEST_HPP
#define RMF_TRAFFIC__RESERVATION_REQUEST_HPP

#include <optional>
#include <vector>
#include <string>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace reservations {

using RequestId = uint64_t;

///=============================================================================
/// \brief This class represents a reservation request.
class ReservationRequest
{

public:
  ///===========================================================================
  /// \brief Time range for describing the starting time. The starting time may
  /// either have a lower bound, upper bound, both or none. This constraints the
  /// possible starting times for a given reservation.
  class TimeRange
  {
  public:
    const std::optional<Time> lower_bound() const;
    const std::optional<Time> upper_bound() const;

    static TimeRange make_time_range(
      std::optional<Time> lower_bound,
      std::optional<Time> upper_bound
    );

    class Implementation;

  private:
    TimeRange();
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  ///===========================================================================
  /// \brief Time range for describing the starting time.
  const std::optional<TimeRange> start_time() const;

  ///===========================================================================
  /// \brief Minimum duration for which the reservation must be held.
  const std::optional<Duration> duration() const;

  ///===========================================================================
  /// \brief The finish time for the reservation. If the finish time is not set.
  const std::optional<Time> finish_time() const;

  ///===========================================================================
  /// \brief Thre resource name
  const std::string resource_name() const;

  ///===========================================================================
  /// \brief \returns true if this is an indefinite reservation. An indefinite
  /// reservation is a reservation that does not have a finish time or a
  /// duration.
  bool is_indefinite() const;

  ///===========================================================================
  /// \brief Create a reservation request.
  /// \param[in] resource_name The name of the resource to reserve.
  /// \param[in] start The time range constraints for the start time of the
  ///   reservation.
  /// \param[in] duration The duration of the reservation.
  /// \param[in] finish The finish time for the reservation.
  /// If both duration and finish are `std::nullopt` then the reservation is
  /// indefinite. If both duration and finish are not `std::nullopt` then the
  /// max(start_time + duration, finish_time) is used as the actual finish time.
  static ReservationRequest make_request(
    std::string resource_name,
    std::optional<TimeRange> start = std::nullopt,
    std::optional<Duration> duration = std::nullopt,
    std::optional<Time> finish = std::nullopt
  );

  class Implementation;

private:
  ReservationRequest();
  rmf_utils::impl_ptr<Implementation> _pimpl;
};
}
}

#endif