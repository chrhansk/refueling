#include "max_clique.hh"

#include "log.hh"

#include "graph/vertex_set.hh"

bool find_clique(const Graph& graph,
                 const VertexFunc<double>& weights,
                 const VertexSet& forbidden_vertices,
                 std::vector<Vertex>& clique,
                 double min_weight,
                 Vertex source)
{
  double current_weight = weights(source);

  VertexSet considered(graph);
  std::vector<Vertex> current_clique{source};
  considered.insert(source);

  Vertex current = source;

  while(true)
  {
    std::vector<Vertex> candidates;

    auto add_vertex = [&](const Vertex& vertex)
                      {
                        if(considered.contains(vertex) || forbidden_vertices.contains(vertex))
                        {
                          return;
                        }

                        for(const Vertex& current_vertex : current_clique)
                        {
                          if(!graph.are_adjacent(vertex, current_vertex))
                          {
                            considered.insert(vertex);
                            return;
                          }
                        }

                        candidates.push_back(vertex);
                      };

    for(const Edge& outgoing : graph.get_outgoing(current))
    {
      add_vertex(outgoing.get_target());
    }

    for(const Edge& incoming : graph.get_incoming(current))
    {
      add_vertex(incoming.get_source());
    }

    if(candidates.empty())
    {
      break;
    }

    Vertex max_candidate;
    double max_weight = -inf;

    for(const Vertex& candidate: candidates)
    {
      if(weights(candidate) > max_weight)
      {
        max_weight = weights(candidate);
        max_candidate = candidate;
      }
    }

    current_clique.push_back(max_candidate);
    current_weight += max_weight;
  }

  if(cmp::gt(current_weight, min_weight))
  {
    Log(debug) << "Found a clique of weight " << current_weight
               << ", size " << current_clique.size();

    if(debugging_enabled())
    {
      for(const auto& first : current_clique)
      {
        for(const auto& second : current_clique)
        {
          if(first == second)
          {
            continue;
          }

          assert(graph.are_adjacent(first, second));
        }
      }

      double actual_weight = 0.;

      for(const auto& vertex : current_clique)
      {
        actual_weight += weights(vertex);
      }

      assert(cmp::eq(current_weight, actual_weight));
    }

    clique = current_clique;

    return true;
  }


  return false;
}


std::vector<std::vector<Vertex>> find_cliques(const Graph& graph,
                                              const VertexFunc<double>& weights,
                                              double min_weight)
{
  std::vector<std::vector<Vertex>> cliques;

  std::vector<Vertex> clique;

  VertexSet forbidden_vertices(graph);

  for(const Vertex& vertex: graph.get_vertices())
  {
    if(forbidden_vertices.contains(vertex))
    {
      continue;
    }

    if(find_clique(graph, weights, forbidden_vertices, clique, min_weight, vertex))
    {
      cliques.push_back(clique);

      for(const Vertex& other : clique)
      {
        forbidden_vertices.insert(other);
      }

      clique.clear();
    }

    assert(clique.empty());
  }


  Log(info) << "Found " << cliques.size() << " cliques";

  return cliques;
}
