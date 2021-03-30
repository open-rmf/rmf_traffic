#include <rmf_utils/catch.hpp>
#include "../../../src/rmf_traffic/reservations/internal_DatabasImpl.hpp"

using namespace rmf_traffic::reservations;


class YayaPapayaNegotiator: public Negotiator
{
public:
    bool offer_received(Reservation res) override
    {
        return true;
    }

    virtual float added_cost(Reservation res) override
    {
        return 0;
    }

    /// Commits to the last offer. 
    virtual void offer_commited() override
    {

    }

    /// Called when we whish to retract the last offer
    virtual void offer_retracted() override
    {

    }

    /// Called when all offers are rejected. I.E none of the proposed
    /// Reservations are acceptable;
    virtual void offers_rejected() 
    {

    }
};
SCENARIO("Reservation test")
{
    auto negotiator = std::make_shared<YayaPapayaNegotiator>();
    Database::Implementation impl;
/*
    GIVEN("An empty database")
    rmf_traffic::Time::
    ReservationRequest req = 
        ReservationRequest::make_request(
            "resource",
            0,
            Reservation::m
        );*/
}
