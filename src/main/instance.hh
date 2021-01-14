#ifndef INSTANCE_HH
#define INSTANCE_HH

#include "index_map.hh"
#include "point.hh"
#include "request.hh"

template <class T>
using RequestMap = IndexMap<Request, T>;

class Instance
{
private:
  Point origin;
  std::vector<Request> requests;

public:
  Instance(const Point& origin,
           const std::vector<Request>& requests)
    : origin(origin),
      requests(requests)
  {}

  Instance()
  {}

  const Point& get_origin() const
  {
    return origin;
  }

  void set_origin(const Point& point)
  {
    origin = point;
  }

  const std::vector<Request>& get_requests() const
  {
    return requests;
  }

  template <class T>
  RequestMap<T> request_map(const T& value = T()) const
  {
    return RequestMap<T>(get_requests().size(), value);
  }

  std::vector<Request>& get_requests()
  {
    return requests;
  }

  void set_requests(const std::vector<Request>& reqs)
  {
    requests = reqs;
  }

};


#endif /* INSTANCE_HH */
