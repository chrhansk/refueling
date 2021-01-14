#include "edge_fuel_difference.hh"

#include <cmath>
#include <stdexcept>

EdgeFuelDifference::EdgeFuelDifference(const Parameters& parameters,
                                       const EdgeMap<OpType>& edge_types,
                                       const EdgeFunc<Units::Time>& travel_times,
                                       const EdgeMap<Request>& edge_requests)
  : edge_types(edge_types),
    travel_times(travel_times),
    edge_requests(edge_requests),
    fuel_difference(parameters)
{

}

Units::Mass EdgeFuelDifference::operator()(const Edge& edge,
                                           const Units::Mass& final_fuel_amount) const
{
  Units::Mass zero(0 * Units::SI::kilogram);

  Units::Time travel_time = travel_times(edge);

  switch(edge_types(edge))
  {
  case OpType::BASE_REFUELING:
    return zero;
  case OpType::BASE_WAITING:
    return final_fuel_amount;
  case OpType::FLIGHT:
    return fuel_difference.flight_initial_fuel(travel_time, final_fuel_amount);
  case OpType::DESCENT:
    return fuel_difference.descent_initial_fuel(travel_time, final_fuel_amount);
  case OpType::REFUELING:
    return refueling_initial_fuel(edge, final_fuel_amount);
  case OpType::CLIMB:
    return fuel_difference.climb_initial_fuel(travel_time, final_fuel_amount);
  }

  throw std::logic_error("Could not determine edge type");

  return zero;
}

Units::Mass EdgeFuelDifference::refueling_initial_fuel(const Edge& edge,
                                                       const Units::Mass& final_fuel_amount) const
{
  return fuel_difference.refueling_initial_fuel(edge_requests(edge).get_amount(), final_fuel_amount);
}

Units::Mass EdgeFuelDifference::initial_fuel(const Path& path,
                                             const Units::Mass& final_fuel_amount) const
{
  auto it = path.get_edges().rbegin();
  auto end = path.get_edges().rend();

  Units::Mass current_fuel_amount = final_fuel_amount;

  for(; it != end; ++it)
  {
    const Edge& edge = *it;

    current_fuel_amount = (*this)(edge, current_fuel_amount);
  }

  return current_fuel_amount;
}


bool EdgeFuelDifference::valid_path(const Path& path,
                                    const Units::Mass& upper_bound,
                                    const Units::Mass& final_fuel_amount) const
{
  if(final_fuel_amount > upper_bound)
  {
    return false;
  }

  auto it = path.get_edges().rbegin();
  auto end = path.get_edges().rend();

  Units::Mass current_fuel_amount = final_fuel_amount;

  for(; it != end; ++it)
  {
    const Edge& edge = *it;

    current_fuel_amount = (*this)(edge, current_fuel_amount);

    if(current_fuel_amount > upper_bound)
    {
      return false;
    }
  }

  return true;
}
