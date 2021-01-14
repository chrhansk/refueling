#ifndef FLOW_GRAPH_HH
#define FLOW_GRAPH_HH

#include "instance.hh"
#include "parameters.hh"

#include "op_type.hh"

#include "graph/graph.hh"
#include "graph/edge_map.hh"
#include "graph/vertex_map.hh"

class FlowGraph
{
private:
  const Instance& instance;
  Parameters parameters;

  Graph graph;

  RequestMap<Vertex> origin_vertices, destination_vertices;
  RequestMap<Edge> takeoff_edges;

  VertexMap<DateTime> times;
  EdgeMap<OpType> edge_types;

  Edge cycle_edge;

  void create_request_vertices();
  void create_request_edges();
  void create_base_edges();

public:
  FlowGraph(const Instance& instance,
            const Parameters& parameters = Parameters());

  operator const Graph&() const
  {
    return graph;
  }

  const RequestMap<Vertex>& get_origin_vertices() const
  {
    return origin_vertices;
  }

  const RequestMap<Vertex>& get_destination_vertices() const
  {
    return destination_vertices;
  }

  const Edge& get_cycle_edge() const
  {
    return cycle_edge;
  }

  const Instance& get_instance() const
  {
    return instance;
  }

  const EdgeMap<OpType>& get_edge_types() const
  {
    return edge_types;
  }
};


#endif /* FLOW_GRAPH_HH */
