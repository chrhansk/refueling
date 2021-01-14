#ifndef PARAMETERS_HH
#define PARAMETERS_HH

#include "util.hh"

class Parameters
{
public:
  class Landing
  {
  private:
    Units::Length distance;
    Units::MassFlowRate fuel_rate;
  public:
    Landing();

    Units::Length get_distance() const
    {
      return distance;
    }

    Units::MassFlowRate get_fuel_rate() const
    {
      return fuel_rate;
    }

  };

  class TakeOff
  {
  private:
    Units::Length distance;
    Units::Length efficiency;

  public:
    TakeOff(const Parameters& parameters);

    Units::Length get_distance() const
    {
      return distance;
    }

    Units::Length get_efficiency() const
    {
      return efficiency;
    }
  };

private:
  //
  Units::Mass takeoff_weight;
  Units::Mass empty_weight;

  Units::Mass refueling_amount;

  Units::Time refueling_duration;
  Units::Time approach_duration;
  Units::Time wet_contact_duration;

  Units::Time base_duration;

  Units::Velocity flight_speed;

  Units::Length efficiency;

  TakeOff takeoff;
  Landing landing;

public:
  Parameters();

  const Landing& get_landing() const
  {
    return landing;
  }

  const TakeOff& get_takeoff() const
  {
    return takeoff;
  }

  Units::Mass get_takeoff_weight() const;
  Units::Mass get_empty_weight() const;

  Units::Mass get_refueling_amount() const;

  Units::Time get_refueling_duration() const;
  Units::Time get_base_duration() const;

  Units::Velocity get_flight_speed() const;

  Units::Length get_efficiency() const;

  Units::Time get_approach_duration() const;
  Units::Time get_wet_contact_duration() const;

  Units::Time get_retreat_duration() const;
};


#endif /* PARAMETERS_HH */
