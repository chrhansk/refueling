#ifndef FUNDAMENTAL_PATH_HH
#define FUNDAMENTAL_PATH_HH

#include "path/path.hh"
#include "program/request_pattern.hh"

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

class FundamentalPath
{
private:
  Path path;
  int index;
  RequestPattern pattern;
  SCIP_VAR* trip_variable;
  TimeSpan timespan;

public:
  FundamentalPath(const Path& path,
                  int index,
                  const RequestPattern& pattern,
                  SCIP_VAR* trip_variable,
                  const TimeSpan& timespan)
    : path(path),
      index(index),
      pattern(pattern),
      trip_variable(trip_variable),
      timespan(timespan)
  {}

  const Path& get_path() const
  {
    return path;
  }

  int get_index() const
  {
    return index;
  }

  const RequestPattern& get_pattern() const
  {
    return pattern;
  }

  SCIP_VAR* get_trip_variable() const
  {
    return trip_variable;
  }

  void set_trip_variable(SCIP_VAR* var)
  {
    trip_variable = var;
  }

  const TimeSpan& get_timespan() const
  {
    return timespan;
  }

  bool conflicts_with(const FundamentalPath& other) const
  {
    return get_timespan().intersects(other.get_timespan());
  }
};


#endif /* FUNDAMENTAL_PATH_HH */
