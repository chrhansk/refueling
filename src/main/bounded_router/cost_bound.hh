#ifndef COST_BOUND_HH
#define COST_BOUND_HH

#include "graph/graph.hh"
#include "graph/edge_set.hh"
#include "graph/vertex_map.hh"

#include "fuel_cost_function.hh"

#include "router/label.hh"

/*
 * The cost bound is based on the assumption that
 * the costs are minimal when the fuel at
 * the end of an edge is zero.
 */
class CostBound : public EdgeFunc<double>
{
private:
  EdgeFuelDifference fuel_difference;
  const FuelCostFunction<double>& costs;
public:
  CostBound(const EdgeFuelDifference& fuel_difference,
            const FuelCostFunction<double>& costs)
    : fuel_difference(fuel_difference),
      costs(costs)
  {}

  double operator()(const Edge& edge) const override
  {
    Units::Mass final_fuel(0 * Units::SI::kilogram);
    Units::Mass initial_fuel = fuel_difference(edge, final_fuel);

    return costs(edge, initial_fuel, final_fuel);
  }

};

class DistanceBound
{
private:
  VertexMap<Label<double>> labels;
  Vertex origin;

public:
  DistanceBound(const Graph& graph,
                const std::vector<Vertex>& topological_ordering,
                const Vertex& origin,
                const EdgeSet& forbidden_edges,
                const EdgeFunc<double>& cost_bound);

  double distance(const Vertex& vertex) const
  {
    assert(reachable(vertex));
    return labels(vertex).get_cost();
  }

  bool reachable(const Vertex& vertex) const
  {
    return labels(vertex).get_state() == State::SETTLED;
  }

  Path path(const Vertex& vertex) const;
};


#endif /* COST_BOUND_HH */
