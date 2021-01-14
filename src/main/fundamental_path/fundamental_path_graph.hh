#ifndef FUNDAMENTAL_PATH_GRAPH_HH
#define FUNDAMENTAL_PATH_GRAPH_HH

#include "instance.hh"
#include "solution.hh"

#include "graph/graph.hh"
#include "graph/edge_map.hh"
#include "graph/vertex_map.hh"

#include "path/path.hh"

#include "op_type.hh"
#include "parameters.hh"
#include "graph/travel_times.hh"

#include "edge_fuel_difference.hh"

class FundamentalPathGraph
{
private:
  const Instance& instance;
  Parameters parameters;
  Graph graph;

  Vertex origin, destination;

  mutable std::vector<Vertex> topological_ordering;

  RequestMap<Edge> request_edges;

  RequestMap<Path> request_paths;

  RequestMap<Edge> climb_edges, descent_edges, refueling_edges;

  EdgeMap<OpType> edge_types;
  EdgeMap<Request> edge_requests;

  VertexMap<DateTime> times;
  VertexMap<Point> coordinates;

  std::multimap<DateTime, Vertex> takeoff_vertices, landing_vertices;

  TravelTimes travel_times;

  void add_request_edges(const Request& request);
  void add_flight_edges();

  void add_waiting_edges();

  Edge add_edge(const Vertex& source,
                const Vertex& target,
                const OpType& type,
                const Request request = Request());

public:
  FundamentalPathGraph(const Instance& instance,
                       const Parameters& parameters = Parameters());

  operator const Graph&() const
  {
    return graph;
  }

  Solution::Path solution_path_for(const Path& path) const;

  std::vector<Path> paths_for(const Solution::Path& solution_path) const;

  const Instance& get_instance() const
  {
    return instance;
  }

  const RequestMap<Edge>& get_request_edges() const
  {
    return request_edges;
  }

  Vertex get_origin() const
  {
    return origin;
  }

  Vertex get_destination() const
  {
    return destination;
  }

  const RequestMap<Path>& get_request_paths() const
  {
    return request_paths;
  }

  const EdgeMap<OpType>& get_edge_types() const
  {
    return edge_types;
  }

  const EdgeMap<Request>& get_edge_requests() const
  {
    return edge_requests;
  }

  const EdgeFunc<Units::Time>& get_travel_times() const
  {
    return travel_times;
  }

  const RequestMap<Edge>& get_climb_edges() const
  {
    return climb_edges;
  }

  const RequestMap<Edge>& get_descent_edges() const
  {
    return descent_edges;
  }

  const RequestMap<Edge>& get_refueling_edges() const
  {
    return descent_edges;
  }

  DateTime get_departure_time(const Request& request) const
  {
    return times(refueling_edges(request).get_source());
  }

  DateTime get_arrival_time(const Request& request) const
  {
    return times(descent_edges(request).get_target());
  }

  const VertexMap<DateTime>& get_times() const
  {
    return times;
  }

  TimeSpan get_timespan(const Path& path) const;

  const std::vector<Vertex>& get_topological_ordering() const;

  const Parameters& get_parameters() const
  {
    return parameters;
  }

  const VertexMap<Point>& get_coordinates() const
  {
    return coordinates;
  }

  EdgeFuelDifference fuel_difference() const
  {
    return EdgeFuelDifference(get_parameters(),
                              get_edge_types(),
                              get_travel_times(),
                              get_edge_requests());
  }
};


#endif /* FUNDAMENTAL_PATH_GRAPH_HH */
