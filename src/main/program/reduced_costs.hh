#ifndef REDUCED_COSTS_HH
#define REDUCED_COSTS_HH

#include "units.hh"

#include "graph/graph.hh"
#include "graph/edge_map.hh"

#include "fuel_cost_function.hh"

class FarkasCosts : public FuelCostFunction<double>
{
private:
  const EdgeFunc<double>& dual_values;
public:
  FarkasCosts(const EdgeFunc<double>& dual_values)
    : dual_values(dual_values)
  {}

  double operator()(const Edge& edge,
                    const Units::Mass& initial_fuel,
                    const Units::Mass& final_fuel) const override;
};

class ReducedCosts : public FuelCostFunction<double>
{
private:
  const EdgeFunc<double>& dual_values;
  const FuelCostFunction<double>& costs;

public:
  ReducedCosts(const FuelCostFunction<double>& costs,
               const EdgeFunc<double>& dual_values)
    : dual_values(dual_values),
      costs(costs)
  {}

  double operator()(const Edge& edge,
                    const Units::Mass& initial_fuel,
                    const Units::Mass& final_fuel) const override;
};

#endif /* REDUCED_COSTS_HH */
