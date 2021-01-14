#ifndef FORBIDDEN_PATH_ROUTER_HH
#define FORBIDDEN_PATH_ROUTER_HH

#include "bounded_router.hh"

#include "bounded_label_set.hh"

#include "cost_bound.hh"

class ForbiddenPathRouter : public BoundedRouter
{
private:

  void create_free_labels(const FuelCostFunction<double>& costs,
                          const DistanceBound& distance_bound,
                          const EdgeSet& forbidden_edges,
                          const std::vector<Path>& forbidden_paths,
                          Vertex origin,
                          Vertex destination,
                          VertexMap<BoundedLabelSet>& labels) const;

public:
  ForbiddenPathRouter(const Graph& graph,
                      const std::vector<Vertex>& topological_ordering,
                      const Parameters& parameters,
                      const EdgeFuelDifference& fuel_difference);

  Result find_paths(const FuelCostFunction<double>& costs,
                    const EdgeSet& forbidden_edges,
                    const std::vector<Path>& forbidden_paths,
                    Vertex origin,
                    Vertex destination,
                    const Settings& settings) const;

  Result find_paths(const FuelCostFunction<double>& costs,
                    const EdgeSet& forbidden_edges,
                    const std::vector<Path>& forbidden_paths,
                    Vertex origin,
                    Vertex destination) const;
};


#endif /* FORBIDDEN_PATH_ROUTER_HH */
