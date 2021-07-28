#ifndef RMF_TRAFFIC__PARKING_HPP
#define RMF_TRAFFIC__PARKING_HPP

#include <rmf_traffic/reservation/Participant.hpp>
#include <rmf_traffic/reservation/Request.hpp>
#include <rmf_traffic/reservation/Adjustment.hpp>

#include <rmf_traffic/Time.hpp>

#include <optional>

namespace rmf_traffic {

class Parking
{
public:

  class Adjustment : public reservation::AbstractAdjustment
  {
  public:



  };

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

  /// Request a place to park by specifying a set of
  class Request : public reservation::TemplateRequest<Parking>
  {
  public:

    Request(std::vector<Alternative> alternatives);

    const std::vector<Alternative>& alternatives() const;

    void adjust(const Adjustment& adjustment) final;

    class Implementation;
  private:
    rmf_utils::impl_ptr<Implementation> _pimpl;
  };

  /// This class is used to describe unsolvable conflicts between the requested
  /// reservations of two or more participants.
  class Contention
  {
  public:

  };

};

} // namespace rmf_traffic

#endif // RMF_TRAFFIC__PARKING_HPP
