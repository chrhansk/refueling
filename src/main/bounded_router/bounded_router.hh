#ifndef BOUNDED_ROUTER_HH
#define BOUNDED_ROUTER_HH

#include "graph/edge_set.hh"

#include "path/path.hh"

#include "edge_fuel_difference.hh"
#include "fuel_cost_function.hh"
#include "request_graph.hh"

#include "bounded_label.hh"
#include "bounded_label_set.hh"

#include "cost_bound.hh"

#include "router/label.hh"

class BoundedRouter
{
public:

  struct Settings
  {
    double cost_bound;
    Units::Mass length_bound;
    double eps;
    bool updated_bound;
    bool heuristic_path_search;

    Settings(const Parameters& parameters)
      : cost_bound(0),
        length_bound(parameters.get_refueling_amount()),
        eps(cmp::eps),
        updated_bound(true),
        heuristic_path_search(true)
    {}

    Settings& with_length_bound(const Units::Mass& value)
    {
      length_bound = value;
      return *this;
    }

    Settings& with_updated_bound(bool value)
    {
      updated_bound = value;
      return *this;
    }

    Settings& with_heuristic_path_search(bool value)
    {
      heuristic_path_search = value;
      return *this;
    }

    Settings& with_tolerance(double val)
    {
      eps = val;
      return *this;
    }

  };

  class Result
  {
  private:
    double min_cost;
    std::vector<Path> paths;

  public:
    Result()
      : min_cost(-inf)
    {}

    const double& get_min_cost() const
    {
      return min_cost;
    }

    const std::vector<Path>& get_paths() const
    {
      return paths;
    }

    double& get_min_cost()
    {
      return min_cost;
    }

    std::vector<Path>& get_paths()
    {
      return paths;
    }

  };

protected:
  const Graph& graph;
  std::vector<Vertex> topological_ordering;
  const Parameters& parameters;
  EdgeFuelDifference fuel_difference;

  Path create_path(std::shared_ptr<BoundedLabel> label,
                   const Vertex& destination) const;

  Result iterate(const FuelCostFunction<double>& costs,
                 const CostBound& cost_bound,
                 const DistanceBound& distance_bound,
                 const EdgeSet& forbidden_edges,
                 VertexMap<BoundedLabelSet>& labels,
                 Vertex origin,
                 Vertex destination,
                 const Settings& settings) const;

public:
  BoundedRouter(const Graph& graph,
                const std::vector<Vertex>& topological_ordering,
                const Parameters& parameters,
                const EdgeFuelDifference& fuel_difference);

  Result find_paths(const FuelCostFunction<double>& costs,
                    const EdgeSet& forbidden_edges,
                    Vertex origin,
                    Vertex destination,
                    const Settings& settings) const;

  Result find_paths(const FuelCostFunction<double>& costs,
                    const EdgeSet& forbidden_edges,
                    Vertex origin,
                    Vertex destination) const;
};


#endif /* BOUNDED_ROUTER_HH */
