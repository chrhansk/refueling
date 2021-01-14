#include "vertex_set.hh"

#include "graph.hh"

VertexSet::VertexSet(const Graph& graph)
  : values(graph.get_vertices().size(), false)
{}

bool VertexSet::contains(const Vertex& vertex) const
{
  return values[vertex.get_index()];
}

void VertexSet::insert(const Vertex& vertex)
{
  values[vertex.get_index()] = true;
}

void VertexSet::remove(const Vertex& vertex)
{
  values[vertex.get_index()] = false;
}

bool VertexSet::contains(const Edge& edge)
{
  return contains(edge.get_source()) && contains(edge.get_target());
}
