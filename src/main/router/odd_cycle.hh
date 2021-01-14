#ifndef ODD_CYCLE_HH
#define ODD_CYCLE_HH

#include "graph/graph.hh"
#include "graph/edge_map.hh"

std::vector<std::vector<Vertex>> find_odd_cycles(const Graph& incompat_graph,
                                                 const EdgeFunc<double>& incompat_weights,
                                                 double max_weight = 1.);

#endif /* ODD_CYCLE_HH */
