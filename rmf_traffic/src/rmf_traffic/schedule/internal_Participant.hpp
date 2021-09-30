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

#include <map>

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

  class Shared
  {
  public:

    Shared(
      const Writer::Registration& registration,
      ParticipantDescription description,
      std::shared_ptr<Writer> writer);

    void retransmit(
      const std::vector<Rectifier::Range>& from,
      ItineraryVersion last_known);

    ItineraryVersion current_version() const;

    ParticipantId get_id() const;

    void correct_id(ParticipantId new_id);

    const ParticipantDescription& get_description() const;

    ~Shared();

  private:
    friend class Participant;

    Writer::Input make_input(std::vector<Route> itinerary);
    ItineraryVersion get_next_version();

    ParticipantId _id;
    ItineraryVersion _version;
    RouteId _last_route_id;
    const ParticipantDescription _description;
    std::shared_ptr<Writer> _writer;
    std::unique_ptr<RectificationRequester> _rectification;

    using ChangeHistory =
      std::map<RouteId, std::function<void()>, rmf_utils::ModularLess<RouteId>>;

    Writer::Input _current_itinerary;

    ChangeHistory _change_history;
    rmf_traffic::Duration _cumulative_delay = std::chrono::seconds(0);
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
