#ifndef RMF_TRAFFIC__RESERVATION__ADJUSTMENT_HPP
#define RMF_TRAFFIC__RESERVATION__ADJUSTMENT_HPP

namespace rmf_traffic {
namespace reservation {

//==============================================================================
class AbstractAdjustment
{
public:
  virtual ~AbstractAdjustment() = 0;
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__ADJUSTMENT_HPP
