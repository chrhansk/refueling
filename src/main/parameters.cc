#include "parameters.hh"


Parameters::Parameters()
  : takeoff_weight(62933 * Units::SI::kilogram),
    empty_weight(14881 * Units::SI::kilogram),
    refueling_amount(42456 * Units::SI::kilogram),
    refueling_duration(20 * Units::Metric::Minute::unit_type()),
    approach_duration(12 * Units::Metric::Minute::unit_type()),
    wet_contact_duration(5 * Units::Metric::Minute::unit_type()),
    base_duration(30 * Units::Metric::Minute::unit_type()),
    flight_speed(240.3 * Units::SI::meter / Units::SI::second),
    efficiency(18393 * Units::Nautical::mile),
    takeoff(*this)
{
  assert(get_approach_duration() + get_wet_contact_duration() <= get_refueling_duration());
}

Parameters::Landing::Landing()
  : distance(156.8 * Units::SI::kilo*Units::SI::meter),
    fuel_rate(160.0 * (Units::SI::kilogram / Units::Metric::Hour::unit_type()))
{}

Parameters::TakeOff::TakeOff(const Parameters& parameters)
  : distance(87.2 * Units::SI::kilo*Units::SI::meter),
    efficiency((7.028) / (19.5) * parameters.get_efficiency())
{}

Units::Mass Parameters::get_takeoff_weight() const
{
  return takeoff_weight;
}

Units::Mass Parameters::get_empty_weight() const
{
  return empty_weight;
}

Units::Mass Parameters::get_refueling_amount() const
{
  return refueling_amount;
}

Units::Time Parameters::get_refueling_duration() const
{
  return refueling_duration;
}

Units::Time Parameters::get_approach_duration() const
{
  return approach_duration;
}

Units::Time Parameters::get_wet_contact_duration() const
{
  return wet_contact_duration;
}

Units::Time Parameters::get_base_duration() const
{
  return base_duration;
}

Units::Velocity Parameters::get_flight_speed() const
{
  return flight_speed;
}

Units::Length Parameters::get_efficiency() const
{
  return efficiency;
}

Units::Time Parameters::get_retreat_duration() const
{
  return get_refueling_duration() - (get_approach_duration() + get_wet_contact_duration());
}
