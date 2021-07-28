#ifndef RMF_TRAFFIC__RESERVATION__ALTERNATIVE_HPP
#define RMF_TRAFFIC__RESERVATION__ALTERNATIVE_HPP

#include <optional>

#include <rmf_utils/impl_ptr.hpp>

#include <rmf_traffic/Time.hpp>

#include <rmf_traffic/reservation/Resource.hpp>

namespace rmf_traffic {
namespace reservation {

//==============================================================================
class Alternative
{
public:

  rmf_traffic::Time earliest_start_time() const;
  Alternative& earliest_start_time(rmf_traffic::Time t);

  std::optional<rmf_traffic::Duration> minimum_duration() const;
  Alternative& minimum_duration(std::optional<rmf_traffic::Duration> d);

  std::optional<rmf_traffic::Time> earliest_finish_time() const;
  Alternative& earliest_finish_time(std::optional<rmf_traffic::Time> t);

  double ideal_cost() const;
  Alternative ideal_cost(double c);




  class Implementation;
private:
  rmf_utils::impl_ptr<Implementation> _pimpl;
};

} // namespace reservation
} // namespace rmf_traffic

#endif // RMF_TRAFFIC__RESERVATION__ALTERNATIVE_HPP
