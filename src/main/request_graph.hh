#ifndef REQUEST_GRAPH_HH
#define REQUEST_GRAPH_HH

#include "instance.hh"

#include "graph/graph.hh"
#include "graph/edge_map.hh"
#include "graph/vertex_map.hh"

#include "path/path.hh"

#include "op_type.hh"
#include "parameters.hh"
#include "graph/travel_times.hh"

#include "edge_fuel_difference.hh"

#include "solution.hh"

class RequestGraph
{

private:
  const Instance& instance;
  Parameters parameters;
  Graph graph;

  VertexMap<Point> coordinates;
  VertexMap<DateTime> times;

  RequestMap<Vertex> origin_vertices;
  RequestMap<Vertex> destination_vertices;

  RequestMap<Edge> takeoff_edges, landing_edges, refueling_edges, request_edges;

  mutable std::vector<Vertex> topological_ordering;

  EdgeMap<OpType> edge_types;
  EdgeMap<Request> edge_requests;

  Vertex origin, destination;

  TravelTimes travel_times;

  Path central_path;

  void create_request_vertices();
  void create_request_edges();

  bool check() const;

public:

  RequestGraph(const Instance& instance,
               const Parameters& parameters = Parameters());

  operator const Graph&() const
  {
    return graph;
  }

  const std::vector<Vertex>& get_topological_ordering() const;

  const Instance& get_instance() const
  {
    return instance;
  }

  const RequestMap<Vertex>& get_origin_vertices() const
  {
    return origin_vertices;
  }

  const RequestMap<Vertex>& get_destination_vertices() const
  {
    return destination_vertices;
  }

  const EdgeMap<OpType>& get_edge_types() const
  {
    return edge_types;
  }

  Path get_central_path() const
  {
    return central_path;
  }

  Path path_for(const Solution::Path& solution_path) const;

  Solution::Path solution_path_for(const Path& path) const;

  Path get_request_path(const Request& request) const;

  const VertexMap<DateTime>& get_times() const
  {
    return times;
  }

  const Request& get_request(const Edge& edge) const;

  const EdgeFunc<Units::Time>& get_travel_times() const
  {
    return travel_times;
  }

  const Parameters& get_parameters() const
  {
    return parameters;
  }

  const Vertex& get_origin() const
  {
    return origin;
  }

  const Vertex& get_destination() const
  {
    return destination;
  }

  Units::Mass delivered_fuel(const Edge& edge) const;

  EdgeFuelDifference fuel_difference() const;

  const EdgeMap<Request>& get_edge_requests() const;

  const VertexMap<Point>& get_coordinates() const
  {
    return coordinates;
  }

  const RequestMap<Edge>& get_request_edges() const
  {
    return request_edges;
  }
};


#endif /* REQUEST_GRAPH_HH */
