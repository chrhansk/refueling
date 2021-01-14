#include "request_graph.hh"

#include "log.hh"
#include "time.hh"

#include "fuel_cost_function.hh"

namespace
{
  struct Connection
  {
    std::vector<Request> arrivals, departures;
  };

}

RequestGraph::RequestGraph(const Instance& instance, const Parameters& parameters)
  : instance(instance),
    parameters(parameters),
    coordinates(graph, Point(0, 0)),
    times(graph, DateTime()),
    origin_vertices(instance.request_map(Vertex())),
    destination_vertices(instance.request_map(Vertex())),
    takeoff_edges(instance.request_map(Edge())),
    landing_edges(instance.request_map(Edge())),
    refueling_edges(instance.request_map(Edge())),
    request_edges(instance.request_map(Edge())),
    edge_types(graph, OpType::FLIGHT),
    edge_requests(graph, Request()),
    travel_times(times)
{
  create_request_vertices();
  create_request_edges();

  std::map<DateTime, Connection> connections;

  const Point origin_point = instance.get_origin();

  const Units::Velocity flight_speed = parameters.get_flight_speed();

  for(const Request& request : instance.get_requests())
  {
    assert(times(origin_vertices(request)) == request.get_time());
    assert(times(destination_vertices(request)) == request.get_time() + parameters.get_refueling_duration());

    Units::Length climb_distance = std::max(origin_point.distance(request.get_origin()),
                                            parameters.get_takeoff().get_distance());

    Units::Length descent_distance = request.get_destination().distance(origin_point);

    Units::Time climb_duration = climb_distance / flight_speed;
    Units::Time descent_duration = descent_distance / flight_speed;

    assert(flight_speed * climb_duration >= parameters.get_takeoff().get_distance());

    DateTime takeoff_time = request.get_time() - climb_duration;
    DateTime landing_time = request.get_time() + parameters.get_refueling_duration() + descent_duration;

    connections[takeoff_time].departures.push_back(request);
    connections[landing_time].arrivals.push_back(request);
  }

  std::map<DateTime, Vertex> base_vertices;

  auto get_base_vertex = [&](const DateTime& time) -> Vertex
    {
      auto it = base_vertices.find(time);

      if(it != std::end(base_vertices))
      {
        return it->second;
      }

      Vertex vertex = graph.add_vertex();

      coordinates.extend(vertex, origin_point);
      times.extend(vertex, time);

      base_vertices.insert(std::make_pair(time, vertex));

      return vertex;
    };

  // create takeoff / landing edges
  for(const auto& pair : connections)
  {
    const Connection& connection = pair.second;

    Vertex vertex = get_base_vertex(pair.first);

    for(const Request& request : connection.departures)
    {
      Edge edge = graph.add_edge(vertex, get_origin_vertices()(request));
      edge_types.extend(edge, OpType::CLIMB);
      edge_requests.extend(edge, Request());

      takeoff_edges(request) = edge;
    }

    for(const Request& request : connection.arrivals)
    {
      Edge edge = graph.add_edge(get_destination_vertices()(request), vertex);
      edge_types.extend(edge, OpType::DESCENT);
      edge_requests.extend(edge, Request());

      landing_edges(request) = edge;
    }
  }

  DateTime origin_time = base_vertices.begin()->first - parameters.get_base_duration();

  origin = get_base_vertex(origin_time);

  // create base refueling edges
  {
    for(const Request& request : instance.get_requests())
    {
      Vertex target = takeoff_edges(request).get_source();
      DateTime target_time = times(target);
      DateTime source_time = target_time - parameters.get_base_duration();

      Vertex source = get_base_vertex(source_time);

      Edge edge = graph.add_edge(source, target);
      edge_types.extend(edge, OpType::BASE_REFUELING);
      edge_requests.extend(edge, Request());

      refueling_edges(request) = edge;
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

      Vertex source = it->second;
      Vertex target = next->second;

      assert(source != target);

      Edge edge = graph.add_edge(source, target);

      assert(coordinates(source) == instance.get_origin());
      assert(coordinates(target) == instance.get_origin());

      edge_types.extend(edge, OpType::BASE_WAITING);
      edge_requests.extend(edge, Request());

      central_path.append(edge);
    }
  }

  destination = base_vertices.rbegin()->second;

  assert(central_path.connects(origin, destination));

  assert(check());

  Log(info) << "Request graph has " << graph.get_vertices().size()
            << " vertices and " << graph.get_edges().size()
            << " edges";

}

bool RequestGraph::check() const
{
  for(const Request& request : instance.get_requests())
  {
    Edge request_edge = request_edges(request);

    std::vector<Edge> edges{request_edge};

    assert(graph.get_outgoing(request_edge.get_source()) == edges);
    assert(graph.get_incoming(request_edge.get_target()) == edges);
  }

  std::vector<Vertex> base_vertices;

  for(const Vertex& vertex : graph.get_vertices())
  {
    if(coordinates(vertex) != instance.get_origin())
    {
      continue;
    }

    base_vertices.push_back(vertex);
  }

  for(const Vertex& vertex : central_path.get_vertices())
  {
    assert(coordinates(vertex) == instance.get_origin());
  }

  return true;
}

const std::vector<Vertex>& RequestGraph::get_topological_ordering() const
{
  if(topological_ordering.empty())
  {
    topological_ordering = graph.get_vertices().collect();

    std::sort(std::begin(topological_ordering),
              std::end(topological_ordering),
              [&](const Vertex& first, const Vertex& second) -> bool
              {
                return get_times()(first) < get_times()(second);
              });

    if(debugging_enabled())
    {
      for(const Edge& edge : graph.get_edges())
      {
        DateTime source_time = get_times()(edge.get_source());
        DateTime target_time = get_times()(edge.get_target());

        assert(source_time <= target_time);
      }
    }

  }

  return topological_ordering;
}

Path RequestGraph::path_for(const Solution::Path& solution_path) const
{
  Path path;

  assert(solution_path.get_origin() == instance.get_origin());
  assert(solution_path.get_destination() == instance.get_origin());

  const auto& steps = solution_path.get_steps();

  std::cout << "Computing path for solution path" << std::endl;

  for(const auto& step : steps)
  {
    if(step.get_request())
    {
      std::cout << "Serving request " << step.get_request()->get_flight()
                << " at " << step.get_timespan().get_begin()
                << std::endl;
    }
  }

  assert(!steps.empty());

  Vertex vertex = get_origin();

  const auto& coordinates = get_coordinates();

  const auto& edge_types = get_edge_types();

  const auto& times = get_times();

  // Start the path
  {
    DateTime begin = solution_path.get_timespan().get_begin();

    for(const auto& edge : central_path.get_edges())
    {
      assert(coordinates(edge.get_source()) == instance.get_origin());
      assert(coordinates(edge.get_target()) == instance.get_origin());

      if(times(vertex) >= begin)
      {
        break;
      }

      path.append(edge);
      vertex = edge.get_target();
    }
  }

  // Process the steps
  for(const auto& step : steps)
  {
    assert(coordinates(vertex) == step.get_origin());
    assert(times(vertex) == step.get_timespan().get_begin());

    bool found = false;

    if(step.get_type() == OpType::BASE_WAITING)
    {
      const auto& central_path = get_central_path();

      auto it = std::begin(central_path.get_edges());
      auto end = std::end(central_path.get_edges());

      bool vertex_found = false;

      for(; it != end; ++it)
      {
        assert(edge_types(*it) == OpType::BASE_WAITING);

        if(it->get_source() == vertex)
        {
          vertex_found = true;
        }

        if(vertex_found)
        {
          path.append(*it);

          if(times(it->get_target()) == step.get_timespan().get_end())
          {
            vertex = it->get_target();
            break;
          }

          assert(times(it->get_target()) < step.get_timespan().get_end());
        }
      }

      continue;
    }

    for(const Edge& edge : graph.get_outgoing(vertex))
    {
      if(edge_types(edge) == step.get_type() &&
         coordinates(edge.get_target()) == step.get_destination() &&
         times(edge.get_target()) == step.get_timespan().get_end())
      {
        assert(!found);

        vertex = edge.get_target();
        path.append(edge);

        found = true;
      }

    }

    assert(found);
  }

  // finish the path...

  {
    for(const Edge& edge : central_path.get_edges())
    {
      if(edge.get_source() == path.get_target())
      {
        path.append(edge);
      }
    }
  }

  assert(path.connects(get_origin(), get_destination()));

  return path;
}

Solution::Path RequestGraph::solution_path_for(const Path& path) const
{
  Solution::Path solution_path;

  Units::Mass zero(0*Units::SI::kilogram);

  EdgeFuelDifference edge_fuel_difference = fuel_difference();

  VertexMap<Units::Mass> fuel(graph, zero);
  EdgeMap<Units::Mass> edge_burned_fuel(graph, zero);

  {
    BurnedFuel burned_fuel(get_edge_requests(), get_edge_types());

    auto it = path.get_edges().rbegin();
    auto end = path.get_edges().rend();

    for(; it != end; ++it)
    {
      const Edge& edge = *it;

      Units::Mass current_fuel = fuel(edge.get_target());

      Units::Mass initial_fuel = edge_fuel_difference(edge, current_fuel);

      fuel(edge.get_source()) = initial_fuel;

      edge_burned_fuel(edge) = burned_fuel(edge, initial_fuel, current_fuel);
    }
  }

  for(const Edge& edge : path.get_edges())
  {
    auto edge_type = edge_types(edge);

    TimeSpan timespan(times(edge.get_source()), times(edge.get_target()));

    if(edge_type == OpType::REFUELING)
    {
      solution_path.append(Solution::Step::refueling(timespan,
                                                     coordinates(edge.get_source()),
                                                     coordinates(edge.get_target()),
                                                     fuel(edge.get_source()),
                                                     fuel(edge.get_target()),
                                                     edge_burned_fuel(edge),
                                                     edge_requests(edge)));
    }
    else
    {
      solution_path.append(Solution::Step::create(edge_type,
                                                  timespan,
                                                  coordinates(edge.get_source()),
                                                  coordinates(edge.get_target()),
                                                  fuel(edge.get_source()),
                                                  fuel(edge.get_target()),
                                                  edge_burned_fuel(edge)));
    }
  }

  return solution_path;
}

Path RequestGraph::get_request_path(const Request& request) const
{
  Path request_path;

  assert(central_path.connects(origin, destination));

  auto it = std::begin(central_path.get_edges());
  auto end = std::end(central_path.get_edges());

  Edge refueling_edge = refueling_edges(request);

  Vertex source = refueling_edges(request).get_source();

  for(;it != end; ++it)
  {
    if(it->get_source() == source)
    {
      break;
    }

    request_path.append(*it);
  }

  request_path.append(refueling_edge);
  request_path.append(takeoff_edges(request));
  request_path.append(request_edges(request));
  request_path.append(landing_edges(request));

  assert(central_path.contains(request_path.get_target()));

  for(; it != end; ++it)
  {
    if(it->get_source() == request_path.get_target())
    {
      break;
    }
  }

  for(; it != end; ++it)
  {
    request_path.append(*it);
  }

  assert(request_path.get_source() == origin);
  assert(request_path.get_target() == destination);
  assert(request_path.connects(origin, destination));

  return request_path;
}

void RequestGraph::create_request_vertices()
{
  for(const Request& request : instance.get_requests())
  {
    Vertex origin_vertex = graph.add_vertex();

    coordinates.extend(origin_vertex, request.get_origin());
    times.extend(origin_vertex, request.get_time());

    Vertex destination_vertex = graph.add_vertex();

    coordinates.extend(destination_vertex, request.get_destination());
    times.extend(destination_vertex, request.get_time() + parameters.get_refueling_duration());

    origin_vertices(request) = origin_vertex;
    destination_vertices(request) = destination_vertex;

    Edge edge = graph.add_edge(origin_vertex, destination_vertex);
    edge_types.extend(edge, OpType::REFUELING);
    edge_requests.extend(edge, request);

    request_edges(request) = edge;
  }
}


void RequestGraph::create_request_edges()
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

      Edge edge = graph.add_edge(destination_vertices(current_request),
                                 origin_vertices(next_request));

      edge_types.extend(edge, OpType::FLIGHT);
      edge_requests.extend(edge, Request());
    }

  }

}

const Request& RequestGraph::get_request(const Edge& edge) const
{
  assert(get_edge_types()(edge) == OpType::REFUELING);

  return edge_requests(edge);
}

Units::Mass RequestGraph::delivered_fuel(const Edge& edge) const
{
  if(get_edge_types()(edge) != OpType::REFUELING)
  {
    return Units::Mass(0*Units::SI::kilogram);
  }

  return edge_requests(edge).get_amount();
}

EdgeFuelDifference RequestGraph::fuel_difference() const
{
  return EdgeFuelDifference(get_parameters(),
                            get_edge_types(),
                            get_travel_times(),
                            get_edge_requests());
}

const EdgeMap<Request>& RequestGraph::get_edge_requests() const
{
  return edge_requests;
}
