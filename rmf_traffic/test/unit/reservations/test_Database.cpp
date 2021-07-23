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
    int priority = 1)
  : _pid(pid),
    _reqid(0),
    _priority(priority)
  {
  }

  bool request_proposal(
    RequestId id,
    Reservation& res,
    uint64_t proposal_version
  ) override
  {
    _proposed[id] = res;
    return true;
  }

  bool request_confirmed(
    RequestId id,
    Reservation& res,
    uint64_t proposal_version
  ) override
  {
    return true;
  }

  bool unassign_request_confirmed(
    RequestId id,
    uint64_t proposal_version
  ) override
  {
    return true;
  }

  bool unassign_request_proposal(
    RequestId id,
    uint64_t proposal_version
  ) override
  {
    _proposed[id] = std::nullopt;
    return true;
  }

  RequestId make_request_blocking(
    std::shared_ptr<Database>& _db,
    std::vector<ReservationRequest>& request_options
  )
  {
    _db->request_reservation(
      _pid,
      _reqid,
      request_options,
      _priority
    );

    return _reqid++;
  }

  std::optional<Reservation> get_proposal(ReservationId rid)
  {
    return _proposed[rid];
  }

  ParticipantId get_id() const
  {
    return _pid;
  }

private:
  ParticipantId _pid;
  RequestId _reqid;

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
      std::make_shared<SimpleParticipant>(0);
    database->register_participant(0, participant1);

    WHEN("the participant makes a request")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+5s
        ),
        {10s}
      );

      auto request1 = std::vector{request1_alt1};
      auto req_id = participant1->make_request_blocking(database, request1);
      auto prop = participant1->get_proposal(req_id);

      THEN("There should be a successful proposal")
      {
        REQUIRE(prop.has_value());
        REQUIRE(prop.value().resource_name() == "table_at_timbre");
      }
    }

    WHEN("A second participant makes a request with a higher priority")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+5s
        ),
        {10s}
      );

      auto request1 = std::vector{request1_alt1};
      auto req_id = participant1->make_request_blocking(database, request1);
      auto prop = participant1->get_proposal(req_id);

      std::shared_ptr<SimpleParticipant> participant2 =
        std::make_shared<SimpleParticipant>(1, 20);
      database->register_participant(1, participant2);
      auto req_id2 = participant2->make_request_blocking(database, request1);
      auto prop2 = participant2->get_proposal(req_id2);
      THEN("The participant with a higher priority should be serviced")
      {
        REQUIRE(prop2.has_value());
        REQUIRE(!participant1->get_proposal(req_id).has_value());
      }
    }

    WHEN("We remove the second participant")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+5s
        ),
        {10s}
      );

      auto request1 = std::vector{request1_alt1};
      auto req_id = participant1->make_request_blocking(database, request1);
      auto prop = participant1->get_proposal(req_id);

      std::shared_ptr<SimpleParticipant> participant2 =
        std::make_shared<SimpleParticipant>(1, 20);
      database->register_participant(1, participant2);
      auto req_id2 = participant2->make_request_blocking(database, request1);
      auto prop2 = participant2->get_proposal(req_id2);
      REQUIRE(prop2.has_value());
      REQUIRE(!participant1->get_proposal(req_id).has_value());

      database->unregister_participant(participant2->get_id());
      THEN("Participant 1 will regain its reservation")
      {
        REQUIRE(participant1->get_proposal(req_id).has_value());
      }
    }


    WHEN("We cancel the reservation of the second participant")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+5s
        ),
        {10s}
      );

      auto request1 = std::vector{request1_alt1};
      auto req_id = participant1->make_request_blocking(database, request1);
      auto prop = participant1->get_proposal(req_id);

      std::shared_ptr<SimpleParticipant> participant2 =
        std::make_shared<SimpleParticipant>(1, 20);
      database->register_participant(1, participant2);
      auto req_id2 = participant2->make_request_blocking(database, request1);
      auto prop2 = participant2->get_proposal(req_id2);
      REQUIRE(prop2.has_value());
      REQUIRE(!participant1->get_proposal(req_id).has_value());

      database->cancel_request(participant2->get_id(), req_id2);
      THEN("Participant 1 will regain its reservation.")
      {
        REQUIRE(participant1->get_proposal(req_id).has_value());
      }
    }

    WHEN("We make three requests")
    {
      auto request1_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now,
          now+5s
        ),
        {10s}
      );
      auto request1 = std::vector{request1_alt1};
      auto req1_id = participant1->make_request_blocking(database, request1);

      REQUIRE(participant1->get_proposal(req1_id).has_value());

      auto request2_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now+10s,
          now+30s
        ),
        {10s}
      );
      auto request2 = std::vector{request2_alt1};
      auto req2_id = participant1->make_request_blocking(database, request2);

      REQUIRE(participant1->get_proposal(req2_id).has_value());

      auto request3_alt1 = ReservationRequest::make_request(
        "table_at_timbre",
        ReservationRequest::TimeRange::make_time_range(
          now+10s,
          now+20s
        ),
        {10s}
      );
      auto request3 = std::vector{request3_alt1};
      auto req3_id = participant1->make_request_blocking(database, request3);
      THEN("All three proposals are accomodated")
      {
        REQUIRE(participant1->get_proposal(req1_id).has_value());
        REQUIRE(participant1->get_proposal(req2_id).has_value());
        REQUIRE(participant1->get_proposal(req3_id).has_value());

        REQUIRE(
          !participant1->get_proposal(req3_id)->conflicts_with(
            participant1->get_proposal(req2_id).value()
        ));

        REQUIRE(
          !participant1->get_proposal(req1_id)->conflicts_with(
            participant1->get_proposal(req2_id).value()
        ));

        REQUIRE(
          !participant1->get_proposal(req3_id)->conflicts_with(
            participant1->get_proposal(req1_id).value()
        ));
      }
    }
  }
}