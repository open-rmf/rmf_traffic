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
 */

#ifndef RMF_TRAFFIC__RESERVATION__NEGOTIATOR_HPP
#define RMF_TRAFFIC__RESERVATION__NEGOTIATOR_HPP

#include <rmf_traffic/reservations/Reservation.hpp>

namespace rmf_traffic {
namespace reservations {

class Negotiator
{
  public:
    /// Called when an offer is made. 
    /// If the offer is updated then this handle will be called again. 
    /// It should @return true if the offer is to be accepted 
    virtual bool offer_received(Reservation res) = 0;

    virtual float added_cost(Reservation res) = 0;

    /// Commits to the last offer. 
    virtual void offer_commited() = 0;

    /// Called when we whish to retract the last offer
    virtual void offer_retracted() = 0;

    /// Called when all offers are rejected. I.E none of the proposed
    /// Reservations are acceptable;
    virtual void offers_rejected() = 0;
};

}
}

#endif