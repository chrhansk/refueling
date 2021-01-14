#include "cost_bound.hh"


DistanceBound::DistanceBound(const Graph& graph,
                             const std::vector<Vertex>& topological_ordering,
                             const Vertex& origin,
                             const EdgeSet& forbidden_edges,
                             const EdgeFunc<double>& cost_bound)
  : labels(graph, Label<double>()),
    origin(origin)
{
  labels(origin) = Label<double>(origin, Edge(origin, origin, inf), 0.);

  auto it = std::begin(topological_ordering);
  auto end = std::end(topological_ordering);

  while(*it != origin && it != end)
  {
    ++it;
  }

  assert(it != end);

  for(; it != end; ++it)
  {
    const Vertex& vertex = *it;

    for(const Edge& outgoing : graph.get_outgoing(vertex))
    {
      if(forbidden_edges.contains(outgoing))
      {
        continue;
      }

      double next_distance = labels(vertex).get_cost() + cost_bound(outgoing);

      if(next_distance < labels(outgoing.get_target()).get_cost())
      {
        Label<double>& label = labels(outgoing.get_target());

        label = Label<double>(outgoing.get_target(),
                              outgoing,
                              next_distance);

        label.set_state(State::LABELED);
      }
    }

    if(labels(vertex).get_state() == State::LABELED)
    {
      labels(vertex).set_state(State::SETTLED);
    }
  }
}

Path DistanceBound::path(const Vertex& vertex) const
{
  assert(reachable(vertex));

  Path path;

  Label<double> current_label = labels(vertex);

  while(current_label.get_vertex() != origin)
  {
    Edge edge = current_label.get_edge();
    path.prepend(edge);

    current_label = labels(edge.get_source());
  }

  return path;
}
