#include <rmf_traffic/reservations/Database.hpp>
#include <set>

namespace rmf_traffic {
namespace reservations {

class ResourceTracker
{
  std::set<Reservation> res;
};

class Database::Implementation
{
public:

};


void Database::make_reservation(
  std::vector<ReservationRequest> request,
  std::shared_ptr<Negotiator> nego)
{
  
}

void Database::set_duration(ReservationId id, rmf_traffic::Duration duration)
{

}

void Database::clear_duration(ReservationId id)
{

}

void Database::set_start_time(ReservationId id, rmf_traffic::Time time)
{

}

void Database::cancel(ReservationId id)
{

}

}
}