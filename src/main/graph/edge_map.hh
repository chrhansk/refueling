#ifndef EDGE_MAP_HH
#define EDGE_MAP_HH

#include "index_map.hh"
#include "graph/graph.hh"

template <class V> using EdgeFunc = Func<Edge, V>;

template<class V>
class EdgeMap : public IndexMap<Edge, V>
{
public:
  EdgeMap(const Graph& graph, V value = V())
    : IndexMap<Edge, V>(graph.get_edges().size(), value)
  {}
};

#endif /* EDGE_MAP_HH */
