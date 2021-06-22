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
#include <rmf_traffic/Time.hpp>
#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace reservations {

using RequestId = uint64_t;

///=============================================================================
/// \brief This class 
class ReservationRequest
{

public:
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

  const std::optional<TimeRange> start_time() const;

  const std::optional<Duration> duration() const;

  const std::optional<Time> finish_time() const;

  const std::string resource_name() const;

  bool is_indefinite() const;

  static ReservationRequest make_request(
    std::string resource_name,
    std::optional<TimeRange> start = std::nullopt,
    std::optional<Duration> duration = std::nullopt,
    std::optional<Time> finish = std::nullopt
  );

  class Implementation;

private:
  ReservationRequest();
  rmf_utils::impl_ptr<Implementation>  _pimpl;
};
}
}

#endif