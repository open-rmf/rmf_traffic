#ifndef RMF_TRAFFIC__RESERVATION__DETAIL__IMPL_NEGOTIATOR_HPP
#define RMF_TRAFFIC__RESERVATION__DETAIL__IMPL_NEGOTIATOR_HPP

#include <rmf_traffic/reservation/Negotiator.hpp>

namespace rmf_traffic {
namespace reservation {

//==============================================================================
namespace detail {
template<typename Resource>
struct CallbackWrapper
{
  using Offer = typename CallbackNegotiator<Resource>::Offer;
  using Contention = typename CallbackNegotiator<Resource>::Contention;

  static AbstractCallbackNegotiator::ConsiderFn consider(
    typename CallbackNegotiator<Resource>::ConsiderFn consider_cb)
  {
    return [consider_cb = std::move(consider_cb)](
      const AbstractOffer& offer,
      AbstractCallbackNegotiator::Response response)
    {
      consider_cb(static_cast<const Offer&>(offer), response);
    };
  }

  static AbstractCallbackNegotiator::AwardFn award(
    typename CallbackNegotiator<Resource>::AwardFn award_cb)
  {
    return [award_cb = std::move(award_cb)](
      const AbstractOffer& offer)
    {
      award_cb(static_cast<const Offer&>(offer));
    };
  }

  static AbstractCallbackNegotiator::FaultFn fault(
    typename CallbackNegotiator<Resource>::FaultFn fault_cb)
  {
    return [fault_cb = std::move(fault_cb)](
      const AbstractContention& contention)
    {
      fault_cb(static_cast<const Contention&>(contention));
    };
  }
};
} // namespace detail

//==============================================================================
template<typename Resource>
CallbackNegotiator<Resource>::CallbackNegotiator(
  ConsiderFn consider_cb,
  AwardFn award_cb,
  FaultFn fault_cb)
: _impl(
    detail::CallbackWrapper<Resource>::consider(std::move(consider_cb)),
    detail::CallbackWrapper<Resource>::award(std::move(award_cb)),
    detail::CallbackWrapper<Resource>::fault(std::move(fault_cb)))
{
  // Do nothing
}

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__DETAIL__IMPL_NEGOTIATOR_HPP
