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

#ifndef SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PARTICIPANT_HPP
#define SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PARTICIPANT_HPP

#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_utils/Modular.hpp>
#include <rmf_utils/RateLimiter.hpp>

#include "internal_Progress.hpp"

namespace rmf_traffic {
namespace schedule {

//==============================================================================
class Participant::Implementation
{
public:

  static Participant make(
    ParticipantDescription description,
    std::shared_ptr<Writer> writer,
    std::shared_ptr<RectificationRequesterFactory> rectifier_factory);

  class Shared : public std::enable_shared_from_this<Shared>
  {
  public:

    Shared(
      const Writer::Registration& registration,
      ParticipantDescription description,
      std::shared_ptr<Writer> writer);

    bool set(PlanId plan, std::vector<Route> itinerary);

    void extend(std::vector<Route> additional_routes);

    void delay(Duration delay);

    void reached(PlanId plan, RouteId route, CheckpointId checkpoint);

    void clear();

    void retransmit(
      const std::vector<Rectifier::Range>& from,
      ItineraryVersion last_known_itinerary,
      ProgressVersion last_known_progress);

    ItineraryVersion current_version() const;

    ParticipantId get_id() const;

    void correct_id(ParticipantId new_id);

    const ParticipantDescription& get_description() const;

    ~Shared();

  private:
    friend class Participant;

    ItineraryVersion get_next_version();

    ParticipantId _id;
    ItineraryVersion _version;
    const ParticipantDescription _description;
    std::shared_ptr<Writer> _writer;
    std::unique_ptr<RectificationRequester> _rectification;

    using ChangeHistory =
      std::map<RouteId, std::function<void()>, rmf_utils::ModularLess<RouteId>>;

    PlanId _current_plan_id;
    Writer::StorageId _next_storage_base;
    Itinerary _current_itinerary;

    ChangeHistory _change_history;
    Duration _cumulative_delay = std::chrono::seconds(0);

    Progress _progress;
    ProgressBuffer _buffered_progress;

    rmf_utils::RateLimiter _version_mismatch_limiter;

    AssignIDPtr _assign_plan_id;
  };

  // Note: It would be better if this constructor were private, but we need to
  // make it public so it can be used by rmf_utils::make_unique_impl
  Implementation(
    const Writer::Registration& registration,
    ParticipantDescription description,
    std::shared_ptr<Writer> writer);

private:
  friend class Participant;
  std::shared_ptr<Shared> _shared;
};

} // namespace schedule
} // namespace rmf_traffic

#endif // SRC__RMF_TRAFFIC__SCHEDULE__INTERNAL_PARTICIPANT_HPP
