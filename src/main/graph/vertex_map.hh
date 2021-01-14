#ifndef VERTEX_MAP_HH
#define VERTEX_MAP_HH

#include "index_map.hh"
#include "graph/graph.hh"

template <class V> using VertexFunc = Func<Vertex, V>;

template<class V>
class VertexMap : public IndexMap<Vertex, V>
{
public:
  VertexMap(const Graph& graph, V value = V())
    : IndexMap<Vertex, V>(graph.get_vertices().size(), value)
  {}
};

#endif /* VERTEX_MAP_HH */
