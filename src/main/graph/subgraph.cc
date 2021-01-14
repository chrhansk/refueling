#include "subgraph.hh"

SubGraph::SubGraph(const Graph& parent,
                   const std::vector<Vertex>& vertices)
  : parent(parent),
    originalVertices(graph, Vertex())
{
  for(const Vertex& vertex : vertices)
  {
    originalVertices.extend(graph.add_vertex(), vertex);
  }
}

Vertex SubGraph::original_vertex(Vertex vertex) const
{
  return originalVertices(vertex);
}

Edge SubGraph::add_edge(Vertex source, Vertex target)
{
  return graph.add_edge(source, target);
}
