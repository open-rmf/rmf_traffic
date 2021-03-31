#include <rmf_traffic/reservations/Query.hpp>

namespace rmf_traffic {
namespace reservations {

class Query::Implementation
{
public:
  std::vector<std::string> _resources;
};

const std::vector<std::string> Query::resources() const
{
  return _pimpl->_resources;
}


Query::Query():
  _pimpl(rmf_utils::make_unique_impl<Implementation>())
{

}

Query Query::make_query(
  const std::vector<std::string>& resources)
{
  Query query;
  query._pimpl->_resources = resources;

  return query;
}

}
}