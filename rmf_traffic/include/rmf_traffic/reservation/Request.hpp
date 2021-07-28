#ifndef RMF_TRAFFIC__RESERVATION__REQUEST_HPP
#define RMF_TRAFFIC__RESERVATION__REQUEST_HPP

#include <vector>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_traffic/reservation/Adjustment.hpp>

namespace rmf_traffic {
namespace reservation {

//==============================================================================
template<typename> class Participant;

//==============================================================================
class AbstractRequest
{
public:
  virtual ~AbstractRequest() = 0;

  class Implementation;
private:
  virtual std::unique_ptr<AbstractRequest> _clone() const = 0;
  virtual void _adjust(const AbstractAdjustment& adjustment) = 0;
};

//==============================================================================
template<typename Resource>
class TemplateRequest : public AbstractRequest
{
public:

  using Self = typename Resource::Request;
  using Adjustment = typename Resource::Adjustment;

  virtual void adjust(const Adjustment& adjustment) = 0;

private:
  std::unique_ptr<AbstractRequest> _clone() const final;
  void _adjust(const AbstractAdjustment& adjustment) final;
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__REQUEST_HPP
