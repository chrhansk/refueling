#include "request_pattern.hh"

RequestPattern::RequestPattern(const Instance& instance,
                               const EdgeMap<OpType>& edge_types,
                               const EdgeMap<Request>& edge_requests,
                               const Path& path)
  : RequestPattern(instance)
{
  for(const Edge& edge : path.get_edges())
  {
    if(edge_types(edge) == OpType::REFUELING)
    {
      insert(edge_requests(edge));
    }
  }

}

RequestPattern::RequestPattern(const Instance& instance,
                               const Request& request)
  : RequestPattern(instance)
{
  insert(request);
}

RequestPattern::RequestPattern(const Instance& instance)
  : pattern(instance.get_requests().size())
{

}

RequestPattern RequestPattern::all(const Instance& instance)
{
  RequestPattern pattern(instance);
  pattern.pattern.set();

  return pattern;
}

void RequestPattern::insert(const Request& request)
{
  pattern[request.get_index()] = 1;
}

bool RequestPattern::empty() const
{
  return !pattern.any();
}

bool RequestPattern::contains(const Request& request) const
{
  return !!pattern[request.get_index()];
}

RequestPattern RequestPattern::operator&(const RequestPattern& other) const
{
  RequestPattern result(*this);
  result.pattern &= other.pattern;

  return result;
}
