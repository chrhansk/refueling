#include "odd_cycle.hh"

#include "graph/vertex_map.hh"
#include "router/router.hh"

#include "cmp.hh"

std::vector<std::vector<Vertex>> find_odd_cycles(const Graph& incompat_graph,
                                                 const EdgeFunc<double>& incompat_weights,
                                                 double max_weight)
{
  std::vector<std::vector<Vertex>> cycles;

  Graph cycle_graph(0, {});
  EdgeMap<double> cycle_weights(cycle_graph, 0);

  VertexMap<Vertex> left_vertices(incompat_graph);
  VertexMap<Vertex> right_vertices(incompat_graph);

  VertexMap<Vertex> original_vertices(cycle_graph);

  for(const Vertex& vertex : incompat_graph.get_vertices())
  {
    Vertex left = cycle_graph.add_vertex();
    left_vertices(vertex) = left;

    original_vertices.extend(left, vertex);

    Vertex right = cycle_graph.add_vertex();
    right_vertices(vertex) = right;

    original_vertices.extend(right, vertex);
  }

  for(const Edge& edge : incompat_graph.get_edges())
  {
    const double weight = incompat_weights(edge);
    Vertex source = edge.get_source();
    Vertex target = edge.get_target();

    cycle_weights.extend(cycle_graph.add_edge(left_vertices(source),
                                              right_vertices(target)),
                         weight);

    cycle_weights.extend(cycle_graph.add_edge(right_vertices(target),
                                              left_vertices(source)),
                         weight);

    cycle_weights.extend(cycle_graph.add_edge(right_vertices(source),
                                              left_vertices(target)),
                         weight);

    cycle_weights.extend(cycle_graph.add_edge(left_vertices(target),
                                              right_vertices(source)),
                         weight);
  }

  VertexSet forbidden_vertices(cycle_graph);

  for(const Edge& edge : incompat_graph.get_edges())
  {
    Dijkstra<double> dijkstra(cycle_graph);

    auto result = dijkstra.shortest_path(left_vertices(edge.get_source()),
                                         left_vertices(edge.get_target()),
                                         cycle_weights,
                                         [&](const Edge& edge) -> bool
                                         {
                                           return !edge.intersects(forbidden_vertices);
                                         });

    if(!result.found)
    {
      continue;
    }

    const double weight = result.cost + incompat_weights(edge);

    if(cmp::lt(weight, max_weight))
    {
      // found odd cycle...

      std::vector<Vertex> incompat_cycle;

      for(const Vertex& vertex : result.path.get_vertices())
      {
        Vertex incompat_vertex = original_vertices(vertex);
        incompat_cycle.push_back(incompat_vertex);
      }

      VertexSet incompat_set(incompat_graph);

      bool is_simple = true;

      for(const Vertex& vertex : incompat_cycle)
      {
        if(incompat_set.contains(vertex))
        {
          is_simple = false;
          break;
        }

        incompat_set.insert(vertex);
      }

      if(!is_simple)
      {
        continue;
      }

      assert((incompat_cycle.size() % 2) == 1);

      // remove the edge further considerations
      {
        forbidden_vertices.insert(left_vertices(edge.get_source()));
        forbidden_vertices.insert(left_vertices(edge.get_target()));

        forbidden_vertices.insert(right_vertices(edge.get_source()));
        forbidden_vertices.insert(right_vertices(edge.get_target()));
      }

      cycles.push_back(std::move(incompat_cycle));
    }
  }

  return cycles;
}
