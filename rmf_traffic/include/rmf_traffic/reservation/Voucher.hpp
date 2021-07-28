#ifndef RMF_TRAFFIC__RESERVATION__VOUCHER_HPP
#define RMF_TRAFFIC__RESERVATION__VOUCHER_HPP

#include <rmf_traffic/reservation/Adjustment.hpp>
#include <rmf_traffic/reservation/Request.hpp>

#include <rmf_utils/impl_ptr.hpp>

namespace rmf_traffic {
namespace reservation {

template<typename> class Participant;
template<typename> class Voucher;

//==============================================================================
class AbstractVoucher
{
public:

  void adjust(std::unique_ptr<AbstractAdjustment> adjustment);
  const AbstractRequest& request();
  void cancel();

  class Implementation;
private:
  template<typename> friend class Voucher;
  AbstractVoucher();

  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
template<typename Resource>
class Voucher
{
public:

  using Offer = typename Resource::Offer;
  using Adjustment = typename Resource::Adjustment;
  using Request = typename Resource::Request;

  void adjust(Adjustment adjustment);

  const Request& request() const;

  /// Cancel or finish this reservation request
  void close();

  class Implementation;
private:
  template<typename> friend class Participant;
  Voucher(AbstractVoucher impl);
  AbstractVoucher _impl;
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__VOUCHER_HPP
