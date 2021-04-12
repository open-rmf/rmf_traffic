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
#include <iostream>2
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

      REQUIRE(res.size() == 2);
      REQUIRE(*res[0] == 10min);
      REQUIRE(*res[1] == 40min);
    }
  }
}
