#ifndef FUEL_DIFFERENCE_HH
#define FUEL_DIFFERENCE_HH

#include "units.hh"
#include "instance.hh"
#include "parameters.hh"

class FuelDifference
{
private:
  const Parameters& parameters;

public:
  FuelDifference(const Parameters& parameters)
    : parameters(parameters)
  {}

  Units::Mass descent_initial_fuel(const Units::Time& flight_duration,
                                   const Units::Mass& final_fuel_amount) const;

  Units::Mass climb_initial_fuel(const Units::Time& flight_duration,
                                 const Units::Mass& final_fuel_amount) const;

  Units::Mass flight_initial_fuel(const Units::Time& flight_duration,
                                  const Units::Mass& final_fuel_amount) const;

  Units::Mass flight_initial_fuel(const Units::Time& flight_duration,
                                  const Units::Mass& final_fuel_amount,
                                  const Units::Length& efficiency) const;

  Units::Mass refueling_initial_fuel(const Units::Mass& requested_fuel_amount,
                                     const Units::Mass& final_fuel_amount) const;
};


#endif /* FUEL_DIFFERENCE_HH */
