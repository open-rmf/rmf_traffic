#ifndef RMF_TRAFFIC__RESERVATION__PARTICIPANT_HPP
#define RMF_TRAFFIC__RESERVATION__PARTICIPANT_HPP

#include <rmf_traffic/reservation/Request.hpp>
#include <rmf_traffic/reservation/Voucher.hpp>
#include <rmf_traffic/reservation/Negotiator.hpp>

namespace rmf_traffic {
namespace reservation {

//==============================================================================
class AbstractParticipant
{
public:

  class Implementation;
private:
  AbstractParticipant();
  AbstractVoucher _request(
    const AbstractRequest& description,
    std::shared_ptr<AbstractNegotiator> negotiator);
};

//==============================================================================
template<typename Resource>
class Participant
{
public:

  using Request = typename Resource::Request;
  using Negotiator = TemplateNegotiator<Resource>;

  Voucher<Resource> request(
    const Request& description,
    std::shared_ptr<Negotiator> negotiator);

  class Implementation;
private:
  Participant();
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__PARTICIPANT_HPP
