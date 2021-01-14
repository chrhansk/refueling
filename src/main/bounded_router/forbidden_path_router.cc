#include "forbidden_path_router.hh"

#include "log.hh"

#include "edge_fuel_difference.hh"

#include "router/shortest_path_tree.hh"

namespace
{
  struct PathLabel
  {
    std::vector<Path::ReverseEdgeIterator> iterators;
    BoundedLabelPtr label;
  };

  typedef std::vector<PathLabel> PathLabels;
}

ForbiddenPathRouter::ForbiddenPathRouter(const Graph& graph,
                                         const std::vector<Vertex>& topological_ordering,
                                         const Parameters& parameters,
                                         const EdgeFuelDifference& fuel_difference)
  : BoundedRouter(graph,
                  topological_ordering,
                  parameters,
                  fuel_difference)
{}

void
ForbiddenPathRouter::create_free_labels(const FuelCostFunction<double>& costs,
                                        const DistanceBound& distance_bound,
                                        const EdgeSet& forbidden_edges,
                                        const std::vector<Path>& forbidden_paths,
                                        Vertex origin,
                                        Vertex destination,
                                        VertexMap<BoundedLabelSet>& labels) const
{
  VertexMap<PathLabels> path_labels(graph);

  const idx num_forbidden_paths = forbidden_paths.size();

  std::vector<Path::ReverseEdgeIterator> end_iterators;

  std::vector<Path::ReverseEdgeIterator> begin_iterators;

  for(const Path& forbidden_path : forbidden_paths)
  {
    begin_iterators.push_back(forbidden_path.get_edges().rbegin());
    end_iterators.push_back(forbidden_path.get_edges().rend());
  }

  path_labels(destination).push_back(PathLabel{begin_iterators,
                                               std::make_shared<BoundedLabel>(destination)});

  auto it = topological_ordering.rbegin();
  auto end = topological_ordering.rend();

  while(*it != destination && it != end)
  {
    ++it;
  }

  assert(*it == destination);

  bool found_origin = false;

  idx num_labels = 0;

  for(; it != end; ++it)
  {
    Vertex current_vertex = *it;

    if(current_vertex == origin)
    {
      found_origin = true;

      break;
    }

    for(const auto& path_label : path_labels(current_vertex))
    {
      const auto& current_iterators = path_label.iterators;

      assert(path_label.iterators.size() == num_forbidden_paths);

      auto current_label = path_label.label;

      if(!current_label)
      {
        continue;
      }

      const Units::Mass current_fuel = current_label->get_fuel();
      const double current_cost = current_label->get_cost();

      for(const Edge& incoming : graph.get_incoming(current_vertex))
      {
        const Vertex next_vertex = incoming.get_source();

        const Units::Mass initial_fuel = fuel_difference(incoming, current_fuel);
        const double edge_cost = costs(incoming, initial_fuel, current_fuel);
        const double next_cost = current_cost + edge_cost;

        if(forbidden_edges.contains(incoming) ||
           !distance_bound.reachable(next_vertex))
        {
          continue;
        }

        idx num_next_active = 0;

        std::vector<Path::ReverseEdgeIterator> next_iterators;

        std::shared_ptr<BoundedLabel> next_label = std::make_shared<BoundedLabel>(next_cost,
                                                                                  initial_fuel,
                                                                                  current_label,
                                                                                  incoming);

        for(idx j = 0; j < num_forbidden_paths; ++j)
        {
          auto current_iterator = current_iterators.at(j);
          auto end_iterator = end_iterators.at(j);

          if(current_iterator == end_iterator)
          {
            next_iterators.push_back(end_iterator);
          }
          else
          {
            assert((*current_iterator).get_target() == current_vertex);

            if(incoming == *current_iterator)
            {
              ++current_iterator;
              next_iterators.push_back(current_iterator);

              ++num_next_active;
            }
            else
            {
              next_iterators.push_back(end_iterator);
            }
          }
        }

        if(num_next_active == 0)
        {
          // create free label

          labels(next_vertex).insert(next_label);

          ++num_labels;
        }
        else
        {
          path_labels(next_vertex).push_back(PathLabel{next_iterators, next_label});
        }
      }
    }
  }

  Log(debug) << "Created " << num_labels
             << " labels from forbidden paths";

  assert(found_origin);

}

BoundedRouter::Result
ForbiddenPathRouter::find_paths(const FuelCostFunction<double>& costs,
                                const EdgeSet& forbidden_edges,
                                const std::vector<Path>& forbidden_paths,
                                Vertex origin,
                                Vertex destination,
                                const Settings& settings) const
{
  Log(info) << "Finding paths (forbidden paths: "
            << forbidden_paths.size()
            << ")";

  CostBound cost_bound(fuel_difference, costs);

  DistanceBound distance_bound(graph,
                               topological_ordering,
                               origin,
                               forbidden_edges,
                               cost_bound);

  VertexMap<BoundedLabelSet> labels(graph);

  create_free_labels(costs,
                     distance_bound,
                     forbidden_edges,
                     forbidden_paths,
                     origin,
                     destination,
                     labels);

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

BoundedRouter::Result
ForbiddenPathRouter::find_paths(const FuelCostFunction<double>& costs,
                                const EdgeSet& forbidden_edges,
                                const std::vector<Path>& forbidden_paths,
                                Vertex origin,
                                Vertex destination) const
{
  return find_paths(costs,
                    forbidden_edges,
                    forbidden_paths,
                    origin,
                    destination,
                    Settings(parameters));
}
