#include "edge_set.hh"

#include "graph.hh"

EdgeSet::EdgeSet(const Graph& graph)
  : values(graph.get_edges().size(), false)
{}

bool EdgeSet::contains(const Edge& edge) const
{
  return values[edge.get_index()];
}

void EdgeSet::insert(const Edge& edge)
{
  values[edge.get_index()] = true;
}

void EdgeSet::remove(const Edge& edge)
{
  values[edge.get_index()] = false;
}
