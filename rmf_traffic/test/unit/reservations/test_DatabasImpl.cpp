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

SCENARIO("Reservation test")
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
}
