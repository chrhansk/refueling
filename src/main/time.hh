#ifndef TIME_HH
#define TIME_HH

#include <boost/date_time/posix_time/posix_time.hpp>

#include "units.hh"

namespace Time = boost::posix_time;

typedef Time::ptime DateTime;
typedef Time::time_duration Duration;

Duration infinite_duration();

extern DateTime initial_time;

extern DateTime neg_inf_time, pos_inf_time;

class TimeSpan
{
private:
  DateTime begin, end;

public:
  TimeSpan(const DateTime& begin, const DateTime& end)
    : begin(begin),
      end(end)
  {
    assert(begin < end);
  }

  TimeSpan()
  {}

  bool contains(const DateTime& time) const
  {
    return begin <= time && time < end;
  }

  bool intersects(const TimeSpan& other) const
  {
    return (contains(other.get_begin()) || contains(other.get_end())) ||
      (other.contains(get_begin()) || other.contains(get_end()));
  }

  Duration duration() const
  {
    return end - begin;
  }

  const DateTime& get_begin() const
  {
    return begin;
  }

  const DateTime& get_end() const
  {
    return end;
  }

  void print(std::ostream& out) const;

  void print() const;
};


std::ostream& operator<<(std::ostream& out, const TimeSpan& timespan);

static inline DateTime operator+(const DateTime& lhs,
                                 const Units::Time& rhs)
{
  return lhs + Time::microseconds((long) (rhs / Units::Time(1.0 * Units::SI::micro * Units::SI::second)));
}
static inline DateTime operator-(const DateTime& lhs,
                                 const Units::Time& rhs)
{
  return lhs - Time::microseconds((long) (rhs / Units::Time(1.0 * Units::SI::micro * Units::SI::second)));
}

static inline Units::Time duration_time(const Duration& duration)
{
  return Units::Time(duration.total_microseconds() * Units::SI::micro * Units::SI::second);
}

Duration fractional_seconds(double value);
double fractional_seconds(Duration duration);

void print_time(const DateTime& time);

#endif /* TIME_HH */
