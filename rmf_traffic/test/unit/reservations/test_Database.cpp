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

#include <rmf_utils/catch.hpp>
#include <rmf_traffic/reservations/Database.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>

#include <iostream>

using namespace rmf_traffic;
using namespace rmf_traffic::reservations;
using namespace std::chrono_literals;

///=============================================================================
/// Emulates a dumb participant
class SimpleParticipant : public Participant
{
public:
  SimpleParticipant(
    ParticipantId pid,
    std::shared_ptr<Database> db,
    int priority = 1)
  : _pid(pid),
    _db(db),
    _reqid(0),
    _priority(priority)
  {
  }

  bool request_proposal(
    RequestId id,
    Reservation& res
  ) override
  {
    _received_confirmation[id] = true;
    _proposed[id] = res;
    return true;
  }

  bool request_confirmed(
    RequestId id,
    Reservation& res
  ) override
  {
    return true;
  }

  bool unassign_request_confirmed(
    RequestId id
  ) override
  {
    return true;
  }

  bool unassign_request_proposal(
    RequestId id
  ) override
  {
    _received_cancellation[id] = true;
    return true;
  }

  RequestId make_request_blocking(
    std::vector<ReservationRequest>& request_options
  )
  {
    _received_confirmation[_reqid] = false;
    _received_cancellation[_reqid] = false;
    _db->request_reservation(
      _pid,
      _reqid,
      request_options,
      _priority
    );

    // Busy wait... Bad but oh well...this is a unit test
    while (
      !_received_confirmation[_reqid] &&
      !_received_cancellation[_reqid]) {}

    return _reqid++;
  }

  std::optional<Reservation> get_proposal(ReservationId rid)
  {
    return _proposed[rid];
  }

private:
  ParticipantId _pid;
  RequestId _reqid;
  std::shared_ptr<Database> _db;
  std::unordered_map<RequestId, bool> _received_confirmation;
  std::unordered_map<RequestId, bool> _received_cancellation;
  std::unordered_map<RequestId, std::optional<Reservation>> _proposed;
  int _priority;
};

SCENARIO("Test database behaviour at the start of our lord, the saviour, UNIX")
{
  auto now = std::chrono::steady_clock::now();
  now -= now.time_since_epoch();
  GIVEN("An empty database and a participant")
  {
    std::shared_ptr<Database> database = std::make_shared<Database>();
    std::shared_ptr<SimpleParticipant> participant1 =
      std::make_shared<SimpleParticipant>(0, database);
    database->register_participant(0, participant1);

    WHEN("the participant makes a request")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+10s
        ),
        {10s}
      );

      auto request1 = std::vector{request1_alt1};
      auto req_id = participant1->make_request_blocking(request1);
      std::cout << "Got reqid" << req_id<< std::endl;
      auto prop = participant1->get_proposal(req_id);

      THEN("There should be a successful proposal")
      {
        REQUIRE(prop.has_value());
        REQUIRE(prop.value().resource_name() == "table_at_timbre");
      }
    }
  }
}