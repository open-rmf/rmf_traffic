#include <rmf_utils/catch.hpp>
#include <rmf_traffic/reservations/Reservation.hpp>

using namespace rmf_traffic;
using namespace rmf_traffic::reservations;
using namespace std::chrono_literals;

SCENARIO(
  "Test reservation conflicts_with")
{
  auto now = std::chrono::steady_clock::now();
  GIVEN("Two overlapping reservations for same resource")
  {
    Reservation res1 = Reservation::make_reservation(
      now,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    Reservation res2 = Reservation::make_reservation(
      now + 10s,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    THEN("reservations conflict with each other")
    {
      REQUIRE(res1.conflicts_with(res2));
      REQUIRE(res2.conflicts_with(res1));
    }
  }

  GIVEN("Two non-overlapping reservations for same resource")
  {
    Reservation res1 = Reservation::make_reservation(
      now,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    Reservation res2 = Reservation::make_reservation(
      now + 20s,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    THEN("reservations do not conflict with each other")
    {
      REQUIRE(!res1.conflicts_with(res2));
      REQUIRE(!res2.conflicts_with(res1));
    }
  }

  GIVEN("Two overlapping reservations for different resource")
  {
    Reservation res1 = Reservation::make_reservation(
      now,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    Reservation res2 = Reservation::make_reservation(
      now + 10s,
      "table_at_timbre",
      20s,
      std::nullopt
    );
    THEN("reservations do not conflict with each other")
    {
      REQUIRE(!res1.conflicts_with(res2));
      REQUIRE(!res2.conflicts_with(res1));
    }
  }

  GIVEN("One reservation")
  {
    Reservation res1 = Reservation::make_reservation(
      now,
      "table_at_koufu",
      20s,
      std::nullopt
    );
    THEN("reservation conflicts with itself")
    {
      REQUIRE(res1.conflicts_with(res1));
    }
  }
}
