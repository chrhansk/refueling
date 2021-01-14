#include "flow_graph.hh"

#include "log.hh"

namespace
{
  struct Connection
  {
    std::vector<Request> arrivals, departures;
  };

}

FlowGraph::FlowGraph(const Instance& instance,
                     const Parameters& parameters)
  : instance(instance),
    parameters(parameters),
    origin_vertices(instance.request_map(Vertex())),
    destination_vertices(instance.request_map(Vertex())),
    takeoff_edges(instance.request_map(Edge())),
    times(graph),
    edge_types(graph, OpType::FLIGHT)
{
  create_request_vertices();
  create_request_edges();
  create_base_edges();

  Log(info) << "Flow graph has " << graph.get_vertices().size()
            << " vertices and " << graph.get_edges().size()
            << " edges";
}


void FlowGraph::create_request_vertices()
{
  for(const Request& request : instance.get_requests())
  {
    Vertex origin_vertex = graph.add_vertex();

    origin_vertices(request) = origin_vertex;
    times.extend(origin_vertex, request.get_time());

    Vertex destination_vertex = graph.add_vertex();

    destination_vertices(request) = destination_vertex;
    times.extend(destination_vertex, request.get_time() + parameters.get_refueling_duration());
  }
}

void FlowGraph::create_request_edges()
{
  auto it = std::begin(instance.get_requests());
  auto end = std::end(instance.get_requests());

  for(; it != end; ++it)
  {
    auto next = it;
    ++next;

    if(next == end)
    {
      continue;
    }

    const Request& current_request = *it;

    for(; next != end; ++next)
    {
      const Request& next_request = *next;

      assert(current_request.get_time() <= next_request.get_time());

      DateTime finish_time = current_request.get_time() + parameters.get_refueling_duration();

      Units::Length distance = current_request.get_destination().distance(next_request.get_origin());
      Units::Time travel_time = distance / parameters.get_flight_speed();

      if(next_request.get_time() < (finish_time + travel_time))
      {
        continue;
      }

      Edge edge = graph.add_edge(destination_vertices(current_request), origin_vertices(next_request));
      edge_types.extend(edge, OpType::FLIGHT);
    }
  }
}

void FlowGraph::create_base_edges()
{
  std::map<DateTime, Connection> connections;

  Point origin_point = instance.get_origin();

  Units::Velocity flight_speed = parameters.get_flight_speed();

  for(const Request& request : instance.get_requests())
  {
    assert(times(origin_vertices(request)) == request.get_time());
    assert(times(destination_vertices(request)) == request.get_time() + parameters.get_refueling_duration());

    Units::Length incoming_distance = std::max(origin_point.distance(request.get_origin()),
                                               parameters.get_takeoff().get_distance());
    Units::Length outgoing_distance = request.get_destination().distance(origin_point);

    Units::Time incoming_time = incoming_distance / flight_speed;
    Units::Time outgoing_time = outgoing_distance / flight_speed;

    assert(flight_speed * incoming_time >= parameters.get_takeoff().get_distance());

    DateTime departure_time = request.get_time() - incoming_time;
    DateTime arrival_time = request.get_time() + parameters.get_refueling_duration() + outgoing_time;

    connections[departure_time].departures.push_back(request);
    connections[arrival_time].arrivals.push_back(request);
  }

  std::map<DateTime, Vertex> base_vertices;

  // create takeoff / landing edges
  for(const auto& pair : connections)
  {
    const Connection& connection = pair.second;

    Vertex vertex = graph.add_vertex();

    times.extend(vertex, pair.first);

    base_vertices.insert(std::make_pair(pair.first, vertex));

    for(const Request& request : connection.departures)
    {
      Edge edge = graph.add_edge(vertex, get_origin_vertices()(request));
      edge_types.extend(edge, OpType::CLIMB);

      takeoff_edges(request) = edge;
    }

    for(const Request& request : connection.arrivals)
    {
      Edge edge = graph.add_edge(get_destination_vertices()(request), vertex);
      edge_types.extend(edge, OpType::CLIMB);
    }
  }

  Vertex origin = graph.add_vertex();

  times.extend(origin, base_vertices.begin()->first - parameters.get_base_duration());

  base_vertices.insert(std::make_pair(times(origin), origin));

  // create refueling edges
  {
    for(const Request& request : instance.get_requests())
    {
      Vertex target = takeoff_edges(request).get_source();

      DateTime target_time = times(target);
      DateTime source_time = target_time - parameters.get_base_duration();
      auto it = base_vertices.lower_bound(source_time);

      assert(it != std::end(base_vertices));

      while(it != std::begin(base_vertices) && times(it->second) > source_time)
      {
        --it;
      }

      assert(times(it->second) <= source_time);

      Vertex source = it->second;

      Edge edge = graph.add_edge(source, target);
      edge_types.extend(edge, OpType::BASE_REFUELING);
    }
  }

  // create waiting edges
  {
    auto it = std::begin(base_vertices);
    auto end = std::end(base_vertices);

    for(; it != end; ++it)
    {
      auto next = it;
      ++next;

      if(next == end)
      {
        continue;
      }

      Edge edge = graph.add_edge(it->second, next->second);
      edge_types.extend(edge, OpType::BASE_WAITING);
    }
  }

  Vertex destination = base_vertices.rbegin()->second;

  cycle_edge = graph.add_edge(destination, origin);
  edge_types.extend(cycle_edge, OpType::BASE_WAITING);
}
