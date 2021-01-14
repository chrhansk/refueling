#ifndef TRAVEL_TIMES_HH
#define TRAVEL_TIMES_HH

#include "units.hh"
#include "time.hh"

class TravelTimes : public EdgeFunc<Units::Time>
{
private:
  const VertexFunc<DateTime>& times;

public:
  TravelTimes(const VertexFunc<DateTime>& times)
    : times(times)
  {}

  Units::Time operator()(const Edge& edge) const override
  {
    return duration_time(times(edge.get_target()) - times(edge.get_source()));
  }

};


#endif /* TRAVEL_TIMES_HH */
