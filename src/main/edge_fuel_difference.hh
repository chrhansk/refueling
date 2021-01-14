#ifndef EDGE_FUEL_DIFFERENCE_HH
#define EDGE_FUEL_DIFFERENCE_HH

#include "fuel_difference.hh"
#include "units.hh"

#include "graph/edge_map.hh"
#include "path/path.hh"

#include "op_type.hh"

class EdgeFuelDifference
{
private:
  const EdgeMap<OpType>& edge_types;
  const EdgeFunc<Units::Time>& travel_times;
  const EdgeMap<Request>& edge_requests;

  FuelDifference fuel_difference;

  Units::Mass refueling_initial_fuel(const Edge& edge,
                                     const Units::Mass& final_fuel_amount) const;

public:
  EdgeFuelDifference(const Parameters& parameters,
                     const EdgeMap<OpType>& edge_types,
                     const EdgeFunc<Units::Time>& travel_times,
                     const EdgeMap<Request>& edge_requests);

  Units::Mass operator()(const Edge& edge,
                         const Units::Mass& final_fuel_amount) const;

  Units::Mass initial_fuel(const Path& path,
                           const Units::Mass& final_fuel_amount = Units::Mass(0 * Units::SI::kilogram)) const;

  bool valid_path(const Path& path,
                  const Units::Mass& upper_bound,
                  const Units::Mass& final_fuel_amount = Units::Mass(0 * Units::SI::kilogram)) const;

};


#endif /* EDGE_FUEL_DIFFERENCE_HH */
