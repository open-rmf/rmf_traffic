#ifndef RMF_TRAFFIC__RESERVATION__NEGOTIATOR_HPP
#define RMF_TRAFFIC__RESERVATION__NEGOTIATOR_HPP

#include <rmf_traffic/reservation/Offer.hpp>
#include <rmf_traffic/reservation/Contention.hpp>

#include <rmf_utils/impl_ptr.hpp>

#include <functional>

namespace rmf_traffic {
namespace reservation {

//==============================================================================
class AbstractNegotiator
{
public:

  class Implementation;
private:
  virtual void _consider(
    const AbstractOffer& offer,
    std::function<void(std::optional<double>)> response) = 0;

  virtual void _award(const AbstractOffer& award) = 0;

  virtual void _fault(const AbstractContention& contention) = 0;
};

//==============================================================================
template<typename Resource>
class TemplateNegotiator : public AbstractNegotiator
{
public:

  using Offer = typename Resource::Offer;
  using Contention = typename Resource::Contention;
  using Response = std::function<void(std::optional<double>)>;

  virtual void consider(const Offer& offer, Response response) = 0;

  virtual void award(const Offer& offer) = 0;

  virtual void fault(const Contention& contention) = 0;

private:
  void _consider(
    const AbstractOffer& offer,
    std::function<void(std::optional<double>)> response) final;

  void _award(const AbstractOffer& award) final;

  void _fault(const AbstractContention& contention) final;
};

//==============================================================================
template<typename> class CallbackNegotiator;

namespace detail {
class AbstractCallbackNegotiator
{
public:

  using Response = std::function<void(std::optional<double>)>;
  using ConsiderFn = std::function<void(const AbstractOffer&, Response)>;
  using AwardFn = std::function<void(const AbstractOffer&)>;
  using FaultFn = std::function<void(const AbstractContention&)>;

  AbstractCallbackNegotiator(
    ConsiderFn consider_cb,
    AwardFn award_cb,
    FaultFn fault_cb);

  void consider(const AbstractOffer& offer, Response response);
  void award(const AbstractOffer& offer);
  void fault(const AbstractContention& contention);

  class Implementation;
private:

  rmf_utils::impl_ptr<Implementation> _pimpl;
};
} // namespace detail

//==============================================================================
template<typename Resource>
class CallbackNegotiator : public TemplateNegotiator<Resource>
{
public:

  using Offer = typename Resource::Offer;
  using Contention = typename Resource::Contention;

  using Response = std::function<void(std::optional<double>)>;
  using ConsiderFn = std::function<void(const Offer& offer, Response response)>;
  using AwardFn = std::function<void(const Offer& award)>;
  using FaultFn = std::function<void(const Contention& contention)>;

  CallbackNegotiator(
    ConsiderFn consider_cb,
    AwardFn award_cb,
    FaultFn fault_cb);

  void consider(const Offer& offer, Response response) final;
  void award(const Offer& offer) final;
  void fault(const Contention& contention) final;

private:
  detail::AbstractCallbackNegotiator _impl;
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__NEGOTIATOR_HPP
