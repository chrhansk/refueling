#ifndef MAX_CLIQUE_HH
#define MAX_CLIQUE_HH

#include "graph/graph.hh"
#include "graph/vertex_map.hh"

std::vector<std::vector<Vertex>> find_cliques(const Graph& graph,
                                              const VertexFunc<double>& weights,
                                              double min_weight);

#endif /* MAX_CLIQUE_HH */
