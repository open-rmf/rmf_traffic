#include <rmf_traffic/reservations/Query.hpp>

namespace rmf_traffic {
namespace reservations {

class Query::Implementation
{
public:
  std::vector<std::string> _resources;
  std::vector<schedule::ParticipantId> _participants;
};

const std::vector<std::string> Query::resources() const
{
  return _pimpl->_resources;
}

const std::vector<schedule::ParticipantId> Query::participants() const
{
  return _pimpl->_participants;
}

Query::Query():
  _pimpl(rmf_utils::make_unique_impl<Implementation>())
{

}

Query Query::make_query(
  std::vector<std::string>& resources,
  std::vector<schedule::ParticipantId>& participants)
{
  Query query;
  query._pimpl->_resources = resources;
  query._pimpl->_participants = participants;

  return query;
}

}
}