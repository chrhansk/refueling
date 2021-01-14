#ifndef REQUEST_PATTERN_HH
#define REQUEST_PATTERN_HH

#include <boost/dynamic_bitset.hpp>

typedef boost::dynamic_bitset<> BitSet;

#include "instance.hh"
#include "request_graph.hh"

class RequestPattern
{
private:
  BitSet pattern;

public:
  RequestPattern(const Instance& instance,
                 const EdgeMap<OpType>& edge_types,
                 const EdgeMap<Request>& edge_requests,
                 const Path& path);

  RequestPattern(const Instance& instance,
                 const Request& request);

  RequestPattern(const Instance& instance);

  static RequestPattern all(const Instance& instance);

  RequestPattern operator&(const RequestPattern& other) const;

  bool empty() const;

  operator bool() const
  {
    return !empty();
  }

  idx size() const
  {
    return pattern.count();
  }

  void insert(const Request& request);

  RequestPattern& operator-=(const RequestPattern& other)
  {
    pattern -= other.pattern;

    return *this;
  }

  bool operator==(const RequestPattern& other) const
  {
    return pattern == other.pattern;
  }

  bool contains(const Request& request) const;
};


#endif /* REQUEST_PATTERN_HH */
