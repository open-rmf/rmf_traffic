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

#ifndef RMF_TRAFFIC__RESERVATIONS_HPP
#define RMF_TRAFFIC__RESERVATIONS_HPP

#include <optional>
#include <vector>
#include <string>
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace reservations {

using ReservationId = uint64_t;
//==============================================================================
/// \brief The Reservation class represents a proposed solution to a given
/// ReservationRequest.
class Reservation
{
public:

  ///===========================================================================
  /// \brief The time at which the Reservation will start.
  const rmf_traffic::Time start_time() const;

  ///===========================================================================
  /// \brief The minimum duration of the Reservation.
  const std::optional<rmf_traffic::Duration> duration() const;

  ///===========================================================================
  /// \brief The earliest finish time
  const std::optional<rmf_traffic::Time> finish_time() const;

  ///===========================================================================
  /// \brief Shifts the actual finish time
  void set_actual_finish_time(rmf_traffic::Time dur);

  ///===========================================================================
  /// \brief creates a new reservation with a different finish time based on
  /// this reservation.
  Reservation propose_new_finish_time(rmf_traffic::Time dur);

  ///===========================================================================
  /// \brief creates a new reservation with a different start time based on
  /// this reservation.
  Reservation propose_new_start_time(rmf_traffic::Time dur);

  ///===========================================================================
  /// \brief Returns true if the reservation is indefinite. An indefinite
  /// reservation has no duration or finish time.
  bool is_indefinite() const;

  ///===========================================================================
  /// \brief Returns the actual finish time of the reservation. If the
  /// reservation is indefinite then it returns a `std::nullopt`.
  const std::optional<rmf_traffic::Time> actual_finish_time() const;

  ///===========================================================================
  /// \brief Returns the resource which is being reserved by the reservation.
  const std::string resource_name() const;

  ///===========================================================================
  /// \brief Returns a reservation id. Used internally to
  ReservationId reservation_id() const;

  ///===========================================================================
  /// \brief Returns true if two reservations are overlapping.
  bool conflicts_with(const Reservation& other) const;

  ///===========================================================================
  /// \brief Checks equality of two reservations. Note: reservation ids are not
  /// compared.
  bool operator==(const Reservation& other) const;

  ///===========================================================================
  /// \brief Checks inequality of two reservations. Note: reservation ids are
  /// not compared.
  bool operator!=(const Reservation& other) const;

  ///===========================================================================
  /// \brief Creates a reservation.
  /// \param[in] start_time The time at which the reservation will start.
  /// \param[in] resource_name The name of the resource which is being reserved.
  /// \param[in] duration The minimum duration of the reservation.
  /// \param[in] finish_time The earliest finish time.
  /// Note: If both `duration` and `finish_time` are `std::nullopt` then the
  /// reservation is indefinite. If both `duration` and `finish_time` are not
  /// `std::nullopt` then the actual finish time is
  /// $max(finish_time, start_time + duration)$
  static Reservation make_reservation(
    rmf_traffic::Time start_time,
    std::string resource_name,
    std::optional<rmf_traffic::Duration> duration,
    std::optional<rmf_traffic::Time> finish_time);

  class Implementation;

  ///===========================================================================
  /// \brief Constructor
  Reservation();

private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};
} // end namespace reservations
} // end namespace rmf_traffic
#endif
