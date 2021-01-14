#include "fuel_difference.hh"

Units::Mass FuelDifference::descent_initial_fuel(const Units::Time& flight_duration,
                                                 const Units::Mass& final_fuel_amount) const
{
  const Units::Length equivalent_distance = parameters.get_flight_speed() * flight_duration;

  const Units::Length gliding_distance = std::min(equivalent_distance, parameters.get_landing().get_distance());
  const Units::Time gliding_duration = gliding_distance / parameters.get_flight_speed();

  const Units::Mass pre_gliding_fuel_amount = final_fuel_amount + parameters.get_landing().get_fuel_rate() * gliding_duration;

  if(flight_duration > gliding_duration)
  {
    return flight_initial_fuel(flight_duration - gliding_duration, pre_gliding_fuel_amount);
  }
  else
  {
    return pre_gliding_fuel_amount;
  }
}

Units::Mass FuelDifference::climb_initial_fuel(const Units::Time& flight_duration,
                                                 const Units::Mass& final_fuel_amount) const
{
  const Units::Velocity flight_speed = parameters.get_flight_speed();
  const Units::Length equivalent_distance = flight_speed * flight_duration;

  const Units::Length takeoff_distance = parameters.get_takeoff().get_distance();

  //assert(equivalent_distance >= takeoff_distance);

  const Units::Length cruising_distance = equivalent_distance - takeoff_distance;

  const Units::Mass pre_cruising_fuel = flight_initial_fuel(cruising_distance / flight_speed, final_fuel_amount);

  return flight_initial_fuel(takeoff_distance / flight_speed,
                             pre_cruising_fuel,
                             parameters.get_takeoff().get_efficiency());
}

Units::Mass FuelDifference::flight_initial_fuel(const Units::Time& flight_duration,
                                                const Units::Mass& final_fuel_amount) const
{
  return flight_initial_fuel(flight_duration, final_fuel_amount, parameters.get_efficiency());
}

Units::Mass FuelDifference::flight_initial_fuel(const Units::Time& flight_duration,
                                                const Units::Mass& final_fuel_amount,
                                                const Units::Length& efficiency) const
{
  const Units::Length equivalent_distance = parameters.get_flight_speed() * flight_duration;

  const double weight_factor = equivalent_distance / efficiency;

  const Units::Mass empty_weight = parameters.get_empty_weight();

  return ((final_fuel_amount + empty_weight)* exp(weight_factor)) - empty_weight;
}

Units::Mass FuelDifference::refueling_initial_fuel(const Units::Mass& requested_fuel_amount,
                                                   const Units::Mass& final_fuel_amount) const
{
  const Units::Time retreat_duration = parameters.get_retreat_duration();

  const Units::Mass pre_retreat_fuel = flight_initial_fuel(retreat_duration, final_fuel_amount);

  Units::Mass pre_refueling_fuel;

  {
    const Units::Mass empty_weight = parameters.get_empty_weight();

    const Units::Mass refeueling_amount = requested_fuel_amount;

    const Units::Time wet_contact_duration = parameters.get_wet_contact_duration();
    const Units::Length equivalent_distance = parameters.get_flight_speed() * wet_contact_duration;
    const Units::Mass equivalent_mass = refeueling_amount * (parameters.get_efficiency() / equivalent_distance);

    const double weight_factor =  equivalent_distance / parameters.get_efficiency();

    pre_refueling_fuel = ((pre_retreat_fuel + empty_weight + equivalent_mass)* exp(weight_factor))
      - (empty_weight + equivalent_mass);
  }

  const Units::Time approach_duration = parameters.get_approach_duration();

  return flight_initial_fuel(approach_duration, pre_refueling_fuel);
}
