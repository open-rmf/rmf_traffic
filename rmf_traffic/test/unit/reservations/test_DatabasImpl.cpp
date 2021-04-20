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
    impl.debug_reservations(impl._resource_schedules["resource"]);
    auto plan = impl.plan_push_back_reservations(
      impl._resource_schedules["resource"],
      current_time,
      current_time+190s,
      req_id1
    );
    REQUIRE(plan.has_value());

    impl.debug_reservations(*plan);
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

SCENARIO("There is one request enqued")
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


  std::vector<ReservationRequest> requests{req1};
  auto req_id1 = impl.add_request_queue(requests, negotiator);

  WHEN("Given a reservation within valid time range")
  {
    Reservation res1 = Reservation::make_reservation(
      current_time + 150s,
      "resource",
      0,
      {200s},
      {current_time + 150s}
    );
    THEN("Satisfies the request")
    {
      REQUIRE(impl.satisfies(req_id1, res1).has_value());
      REQUIRE(impl.satisfies(req_id1, res1).value() ==0);
    }
  }

  WHEN("Given a reservation that starts too early")
  {
    Reservation res1 = Reservation::make_reservation(
      current_time - 150s,
      "resource",
      0,
      {200s},
      {current_time + 150s}
    );
    THEN("does not satisfy request")
    {
      REQUIRE(!impl.satisfies(req_id1, res1).has_value());
    }
  }

  WHEN("Given a reservation that starts too late")
  {
    Reservation res1 = Reservation::make_reservation(
      current_time + 1050s,
      "resource",
      0,
      {200s},
      {current_time + 150s}
    );
    THEN("does not satisfy request")
    {
      REQUIRE(!impl.satisfies(req_id1, res1).has_value());
    }
  }
}

/*SCENARIO("Reservation test")
{
  auto negotiator = std::make_shared<YayaPapayaNegotiator>();
  Database::Implementation impl;  
  GIVEN("An empty database")
  {
    Database db;  
    WHEN("reserving a resource")
    {
      rmf_traffic::Time current_time = std::chrono::steady_clock::now();
      ReservationRequest req = 
          ReservationRequest::make_request(
              "resource",
              0,
              ReservationRequest::TimeRange::make_time_range(
                  {current_time + 100s},
                  {current_time + 200s}),
              {200s}
          );
      
      db.make_reservation({req}, negotiator);
      THEN("The reservation takes up the earliest possible time")
      {
        Query query = Query::make_query(std::vector<std::string>{"resource"});
        auto x = db.query(query);
        for(auto resource = x.begin(); resource != x.end(); resource++)
        {
          for(
            auto reservation = resource->second.cbegin();
            reservation!= resource->second.cend();
            reservation++)
          {
            REQUIRE(reservation->first == current_time + 100s);
            REQUIRE(reservation->second.actual_finish_time() == current_time + 300s);            
          }
        }
      }
    }
  }
}*/
/*#include <iostream>
SCENARIO("test conflict optimizer")
{
  Database::Implementation::ResourceSchedule schedule;

  WHEN(
    "There are two reservations with a gap of 30min and a request at 10min l8r")
  {
    using namespace std::literals::chrono_literals;
    auto now = std::chrono::steady_clock::now();
    auto next_res_start = now+1h;
    auto res1 = Reservation::make_reservation(
      now,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    auto res2 = Reservation::make_reservation(
      next_res_start,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    schedule.insert({now, res1});
    schedule.insert({res2.start_time(), res2});

    auto req_time = *res2.actual_finish_time() + 10min;
    auto it = ++schedule.begin();
    THEN("First conflict occurs at 10 minutes second conflict happens at 30min")
    {
      auto res = Database::Implementation::nth_conflict_times_bring_forward(
        schedule,
        it,
        req_time,
        now - 1h
      );

      REQUIRE(res.size() == 3);
      REQUIRE(*res[1] == 10min);
      REQUIRE(*res[2] == 40min);
    }
  }

  WHEN(
    "There are two reservations with no gap and a request at 10min l8r")
  {
    using namespace std::literals::chrono_literals;
    auto now = std::chrono::steady_clock::now();
    auto next_res_start = now+30min;
    auto res1 = Reservation::make_reservation(
      now,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    auto res2 = Reservation::make_reservation(
      next_res_start,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    schedule.insert({now, res1});
    schedule.insert({res2.start_time(), res2});

    auto req_time = *res2.actual_finish_time() + 10min;
    auto it = ++schedule.begin();
    THEN("First conflict occurs at 10 minutes second conflict happens at 30min")
    {
      auto res = Database::Implementation::nth_conflict_times_bring_forward(
        schedule,
        it,
        req_time,
        now - 1h
      );

      REQUIRE(res.size() == 3);
      REQUIRE(res[1].has_value() == false);
      REQUIRE(res[2].has_value());
      REQUIRE(res[2] == 10min);
    }
  }

  WHEN("We try to push_back two reservations with no gap")
  {
    using namespace std::literals::chrono_literals;
    auto now = std::chrono::steady_clock::now();
    auto next_res_start = now+30min;
    auto res1 = Reservation::make_reservation(
      now,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    auto res2 = Reservation::make_reservation(
      next_res_start,
      "rubbish",
      0,
      {30min},
      std::nullopt);
    
    schedule.insert({now, res1});
    schedule.insert({res2.start_time(), res2});

    auto req_time = now - 10min;
    auto it = schedule.begin();
    THEN("First conflict occurs at 10 minutes second conflict happens at 30min")
    {
      auto res = Database::Implementation::nth_conflict_times_push_back(
        schedule,
        it,
        req_time,
        now + 2h
      );

      REQUIRE(res.size() == 3);
      REQUIRE(res[1].has_value() == false);
      REQUIRE(res[2].has_value());
      REQUIRE(res[2] == 10min);
    }
  }
}*/
