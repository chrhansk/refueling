#include "reduced_costs.hh"

double FarkasCosts::operator()(const Edge& edge,
                               const Units::Mass& initial_fuel,
                               const Units::Mass& final_fuel) const
{
  return -1. * dual_values(edge);
}

double ReducedCosts::operator()(const Edge& edge,
                                const Units::Mass& initial_fuel,
                                const Units::Mass& final_fuel) const
{
  const double cost = costs(edge, initial_fuel, final_fuel);

  const double dual_value = dual_values(edge);

  return cost - dual_value;
}
