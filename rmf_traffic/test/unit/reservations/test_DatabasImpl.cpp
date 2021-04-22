#include <rmf_utils/catch.hpp>
#include <rmf_traffic/reservations/Singlish.hpp>
#include "../../../src/rmf_traffic/reservations/internal_DatabasImpl.hpp"
#include "../../../src/rmf_traffic/reservations/internal_ViewImpl.hpp"

using namespace rmf_traffic::chope;
using namespace std::chrono_literals;

// http://www.singlish.net/yaya-papaya/
// Too arrogant to reallize the possibility that an offer is wrong
class YayaPapayaNegotiator: public Negotiator
{
public:
  bool offer_received(Reservation res) override
  {
    return true;
  }

  float added_cost(Reservation res) override
  {
    return 0;
  }

  /// Commits to the last offer. 
  void offer_commited() override
  {

  }

  void offer_retracted() override
  {
  
  }


  virtual void offers_rejected() override
  {
    
  }
};


SCENARIO("There is one request")
{
  Database::Implementation impl;

  auto negotiator = std::make_shared<YayaPapayaNegotiator>();

  rmf_traffic::Time current_time = std::chrono::steady_clock::now();

  ReservationRequest req1 = 
      ReservationRequest::make_request(
          "resource",
          0,
          ReservationRequest::TimeRange::make_time_range(
              {current_time + 100s},
              {current_time + 200s}),
          {200s}
      );

  Reservation res1 = Reservation::make_reservation(
    current_time + 150s,
    "resource",
    0,
    {200s},
    {current_time + 150s}
  );

  std::vector<ReservationRequest> requests{req1};
  auto req_id1 = impl.add_request_queue(requests, negotiator);
  impl.associate_request_with_reservation(req_id1, res1);
  
  WHEN("We attempt to push back a request within its fixed time range")
  {
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+190s,
      req_id1
    );
    REQUIRE(plan.has_value());
    REQUIRE(plan.value().push_back_reservations.size() == 1);
    REQUIRE(plan.value().bring_forward_reservations.size() == 0);
    auto new_res =  plan.value().push_back_reservations[0];
    REQUIRE(new_res.reservation_id() == res1.reservation_id());
    REQUIRE(new_res.start_time() == current_time+190s);
    //impl.debug_reservations(*plan);
  }

  WHEN("We attempt to push back a request outside of its fixed time range")
  {
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+500s,
      req_id1
    );

    REQUIRE(!plan.has_value());
  }
}

SCENARIO("There are two requests with no gap")
{
  Database::Implementation impl;

  auto negotiator = std::make_shared<YayaPapayaNegotiator>();

  rmf_traffic::Time current_time = std::chrono::steady_clock::now();
  current_time = current_time-current_time.time_since_epoch();

  ReservationRequest req1 = 
      ReservationRequest::make_request(
          "resource",
          0,
          ReservationRequest::TimeRange::make_time_range(
              {current_time + 100s},
              {current_time + 200s}),
          {200s}
      );

  Reservation res1 = Reservation::make_reservation(
    current_time + 150s,
    "resource",
    0,
    {200s},
    {current_time + 150s}
  );

  std::vector<ReservationRequest> requests{req1};
  auto req_id1 = impl.add_request_queue(requests, negotiator);
  impl.associate_request_with_reservation(req_id1, res1);

  ReservationRequest req2 = 
      ReservationRequest::make_request(
          "resource",
          0,
          ReservationRequest::TimeRange::make_time_range(
              {current_time + 300s},
              {current_time + 500s}),
          {200s}
      );

  Reservation res2 = Reservation::make_reservation(
    current_time + 350s,
    "resource",
    0,
    {200s},
    {current_time + 850s}
  );

  std::vector<ReservationRequest> requests2{req2};
  auto req_id2 = impl.add_request_queue(requests2, negotiator);
  impl.associate_request_with_reservation(req_id2, res2);
  
  WHEN("We attempt to push back a request within its fixed time range")
  {
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+190s,
      req_id1
    );
    REQUIRE(plan.has_value());
    REQUIRE(plan->push_back_reservations.size() == 2);
    REQUIRE(plan->bring_forward_reservations.size() == 0);
  }

  WHEN("We attempt to push back a request outside of its fixed time range")
  {
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+500s,
      req_id1
    );

    REQUIRE(!plan.has_value());
  }


  WHEN("We call nth_conflict_push_back")
  {

    // Here we are pushing n conflict tables back.
    auto conflict_table = impl.nth_conflict_times_push_back(
      impl._resource_schedules["resource"],
      impl._resource_schedules["resource"].begin(),
      current_time
    );
    
    //impl.debug_reservations(impl._resource_schedules["resource"]);

    for(std::size_t i = 0; i < conflict_table.size(); i++)
    {
      std::cout << "Conflict table " << i << ": "
        << (conflict_table[i].has_value() 
          ? conflict_table[i]->count(): -1)
        << std::endl;
    }
  }
}

SCENARIO("There are two requests with a small gap")
{
  Database::Implementation impl;

  auto negotiator = std::make_shared<YayaPapayaNegotiator>();

  rmf_traffic::Time current_time = std::chrono::steady_clock::now();
  current_time = current_time-current_time.time_since_epoch();

  ReservationRequest req1 = 
      ReservationRequest::make_request(
          "resource",
          0,
          ReservationRequest::TimeRange::make_time_range(
              {current_time + 100s},
              {current_time + 200s}),
          {200s}
      );

  Reservation res1 = Reservation::make_reservation(
    current_time + 150s,
    "resource",
    0,
    {200s},
    {current_time + 150s}
  );

  std::vector<ReservationRequest> requests{req1};
  auto req_id1 = impl.add_request_queue(requests, negotiator);
  impl.associate_request_with_reservation(req_id1, res1);

  ReservationRequest req2 = 
      ReservationRequest::make_request(
          "resource",
          0,
          ReservationRequest::TimeRange::make_time_range(
              {current_time + 300s},
              {current_time + 500s}),
          {200s}
      );

  Reservation res2 = Reservation::make_reservation(
    current_time + 400s,
    "resource",
    0,
    {200s},
    {current_time + 850s}
  );

  std::vector<ReservationRequest> requests2{req2};
  auto req_id2 = impl.add_request_queue(requests2, negotiator);
  impl.associate_request_with_reservation(req_id2, res2);
  
  WHEN("We attempt to push back a request within its fixed time range")
  {
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+190s,
      req_id1
    );
    REQUIRE(plan.has_value());
    REQUIRE(plan->push_back_reservations.size() == 1);
    REQUIRE(plan->bring_forward_reservations.size() == 0);
  }

  WHEN("We call nth_conflict_push_back")
  {

    // Here we are pushing n conflict tables back.
    auto conflict_table = impl.nth_conflict_times_push_back(
      impl._resource_schedules["resource"],
      impl._resource_schedules["resource"].begin(),
      current_time
    );
    
    impl.debug_reservations(impl._resource_schedules["resource"]);

    for(std::size_t i = 0; i < conflict_table.size(); i++)
    {
      std::cout << "Conflict table " << i << ": "
        << (conflict_table[i].has_value() 
          ? conflict_table[i]->count(): -1)
        << std::endl;
    }
  }
}


