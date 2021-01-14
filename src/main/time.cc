#include "time.hh"

DateTime initial_time(boost::gregorian::date(2000,boost::gregorian::Jan,1), Duration());

DateTime neg_inf_time(boost::posix_time::neg_infin);
DateTime pos_inf_time(boost::posix_time::pos_infin);

Duration infinite_duration()
{
  return Duration(boost::posix_time::pos_infin);
}

void TimeSpan::print(std::ostream& out) const
{
  out << "[" << get_begin() << ", " << get_end() << ")";
}


void TimeSpan::print() const
{
  print(std::cout);
}


std::ostream& operator<<(std::ostream& out, const TimeSpan& timespan)
{
  timespan.print(out);

  return out;
}

Duration fractional_seconds(double value)
{
  double seconds;
  double fractional = modf(value, &seconds);

  return Time::seconds((long) seconds) + Time::microseconds((long) (fractional * 1e6));
}

double fractional_seconds(Duration duration)
{
  return duration.total_microseconds() / ((double) 1e6);
}

void print_time(const DateTime& time)
{
  std::cout << time << std::endl;
}
