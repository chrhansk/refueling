#include "bounded_router.hh"

#include "log.hh"

#include "edge_fuel_difference.hh"

#include "router/shortest_path_tree.hh"

BoundedRouter::BoundedRouter(const Graph& graph,
                             const std::vector<Vertex>& topological_ordering,
                             const Parameters& parameters,
                             const EdgeFuelDifference& fuel_difference)
  : graph(graph),
    topological_ordering(topological_ordering),
    parameters(parameters),
    fuel_difference(fuel_difference)
{
}

Path BoundedRouter::create_path(BoundedLabelPtr label,
                                const Vertex& destination) const
{
  Path path;

  while(label->get_vertex() != destination)
  {
    path.append(label->get_edge());
    label = label->get_predecessor();
  }

  return path;
}

BoundedRouter::Result
BoundedRouter::iterate(const FuelCostFunction<double>& costs,
                       const CostBound& cost_bound,
                       const DistanceBound& distance_bound,
                       const EdgeSet& forbidden_edges,
                       VertexMap<BoundedLabelSet>& labels,
                       Vertex origin,
                       Vertex destination,
                       const Settings& settings) const
{
  double current_cost_bound = settings.cost_bound;

  Result result;

  std::function<bool(double)> valid_cost;

  if(settings.updated_bound)
  {
    valid_cost = [&](double cost) -> bool
      {
        return cmp::lt(cost, current_cost_bound, settings.eps);
      };
  }
  else
  {
    valid_cost = [&](double cost) -> bool
      {
        return cmp::lt(cost, settings.cost_bound, settings.eps);
      };
  }

  auto it = topological_ordering.rbegin();
  auto end = topological_ordering.rend();

  while(*it != destination && it != end)
  {
    ++it;
  }

  assert(*it == destination);

  bool found_origin = false;

  for(; it != end; ++it)
  {
    Vertex current_vertex = *it;

    if(current_vertex == origin)
    {
      found_origin = true;

      break;
    }

    for(auto current_label : labels(current_vertex).get_labels())
    {
      assert(current_label);

      const Units::Mass current_fuel = current_label->get_fuel();
      const double current_cost = current_label->get_cost();

      for(const Edge& incoming : graph.get_incoming(current_vertex))
      {
        const Vertex next_vertex = incoming.get_source();

        assert(next_vertex != current_vertex);

        if(forbidden_edges.contains(incoming) ||
           !distance_bound.reachable(next_vertex))
        {
          continue;
        }

        const Units::Mass initial_fuel = fuel_difference(incoming, current_fuel);

        if(initial_fuel > settings.length_bound)
        {
          continue;
        }

        const double edge_cost = costs(incoming, initial_fuel, current_fuel);
        const double next_cost = current_cost + edge_cost;

        assert(cmp::ge(edge_cost, cost_bound(incoming)));

        const double head_cost_bound = distance_bound.distance(next_vertex);
        const double total_cost_bound = next_cost + head_cost_bound;

        if(!valid_cost(total_cost_bound))
        {
          continue;
        }

        const Path head_path = distance_bound.path(next_vertex);

        assert(head_path.connects(origin, next_vertex));

        if(settings.heuristic_path_search &&
           fuel_difference.valid_path(head_path, settings.length_bound, initial_fuel))
        {
          Path tail_path = create_path(current_label, destination);
          tail_path.prepend(incoming);

          assert(cmp::rel::eq(next_cost, costs.cost(tail_path, fuel_difference)));

          assert(tail_path.connects(next_vertex, destination));
          assert(fuel_difference.initial_fuel(tail_path) == initial_fuel);

          Path path = head_path;

          path.append(tail_path);

          assert(fuel_difference.valid_path(path, settings.length_bound));
          assert(path.connects(origin, destination));

          const double head_cost = costs.cost(head_path, fuel_difference, initial_fuel);
          const double total_cost = next_cost + head_cost;

          assert(cmp::ge(head_cost, head_cost_bound));

          assert(path.connects(origin, destination));
          assert(fuel_difference.valid_path(path, settings.length_bound));

          assert(path.satisfies([&](const Edge& edge) -> bool
            {
              return !forbidden_edges.contains(edge);
            }));

          if(debugging_enabled())
          {
            const double actual_total_cost = costs.cost(path, fuel_difference);
            assert(cmp::rel::eq(total_cost, actual_total_cost));
          }

          if(valid_cost(total_cost))
          {
            current_cost_bound = std::min(current_cost_bound, total_cost);
            result.get_paths().push_back(path);
          }
        }

        BoundedLabelPtr next_label = std::make_shared<BoundedLabel>(next_cost,
                                                                    initial_fuel,
                                                                    current_label,
                                                                    incoming);

        labels(next_vertex).insert(next_label);
      }
    }
  }

  assert(found_origin);

  for(auto current_label : labels(origin).get_labels())
  {
    double current_cost = current_label->get_cost();

    if(!valid_cost(current_cost))
    {
      continue;
    }

    Path path = create_path(current_label, destination);

    assert(path.connects(origin, destination));
    assert(fuel_difference.valid_path(path, settings.length_bound));

    result.get_paths().push_back(path);

    current_cost_bound = std::min(current_cost_bound, current_label->get_cost());
  }


  if(result.get_paths().empty())
  {
    Log(info) << "Could not find paths";
  }
  else
  {
    result.get_min_cost() = current_cost_bound;

    Log(info) << "Found "
              << result.get_paths().size()
              << " paths, min cost = "
              << current_cost_bound;
  }

  if(debugging_enabled() && !result.get_paths().empty())
  {
    for(const auto& path : result.get_paths())
    {
      assert(fuel_difference.valid_path(path, settings.length_bound));
    }

    double min_cost = inf;

    for(const auto& path : result.get_paths())
    {
      min_cost = std::min(min_cost, costs.cost(path, fuel_difference));
    }

    assert(cmp::rel::eq(min_cost, result.get_min_cost()));
  }


  return result;
}

BoundedRouter::Result
BoundedRouter::find_paths(const FuelCostFunction<double>& costs,
                          const EdgeSet& forbidden_edges,
                          Vertex origin,
                          Vertex destination) const
{
  return find_paths(costs,
                    forbidden_edges,
                    origin,
                    destination,
                    Settings(parameters));
}

BoundedRouter::Result
BoundedRouter::find_paths(const FuelCostFunction<double>& costs,
                          const EdgeSet& forbidden_edges,
                          Vertex origin,
                          Vertex destination,
                          const BoundedRouter::Settings& settings) const
{
  Log(info) << "Finding paths";

  CostBound cost_bound(fuel_difference, costs);

  DistanceBound distance_bound(graph,
                               topological_ordering,
                               origin,
                               forbidden_edges,
                               cost_bound);

  VertexMap<BoundedLabelSet> labels(graph);

  {
    BoundedLabelPtr label = std::make_shared<BoundedLabel>(destination);

    labels(destination).insert(label);
  }

  Result result = iterate(costs,
                          cost_bound,
                          distance_bound,
                          forbidden_edges,
                          labels,
                          origin,
                          destination,
                          settings);

  return result;
}
