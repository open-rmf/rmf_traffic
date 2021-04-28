#ifndef RMF_TRAFFIC__RESERVATIONS__RESERVATION_MANAGER
#define RMF_TRAFFIC__RESERVATIONS__RESERVATION_MANAGER

#include <rmf_traffic/reservations/ReservationRequest.hpp>

namespace rmf_traffic {
namespace reservations {
class AbstractReservationManager
{
  // Register and unregister particpants
public:
  virtual ParticipantId register_participant(Participant participant) = 0;

  virtual void unregister_participant(ParticipantId id) = 0;

  RequestId request_reservation(ParticipantId id,
    std::vector<ReservationRequest>& request) = 0;

  void cancel_request(ParticipantId participant, RequestId request);
};

}
}
#endif