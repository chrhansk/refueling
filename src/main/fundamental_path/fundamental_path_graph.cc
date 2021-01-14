#include "fundamental_path_graph.hh"

#include "log.hh"

#include "fuel_cost_function.hh"

#include "program/request_pattern.hh"

FundamentalPathGraph::FundamentalPathGraph(const Instance& instance,
                                           const Parameters& parameters)
  : instance(instance),
    parameters(parameters),
    request_edges(instance.request_map(Edge())),
    request_paths(instance.request_map(Path())),
    climb_edges(instance.request_map(Edge())),
    descent_edges(instance.request_map(Edge())),
    refueling_edges(instance.request_map(Edge())),
    edge_types(graph, OpType::FLIGHT),
    edge_requests(graph, Request()),
    times(graph, DateTime()),
    coordinates(graph, Point()),
    travel_times(times)
{

  origin = graph.add_vertex();
  times.extend(origin, initial_time);
  coordinates.extend(origin, instance.get_origin());

  destination = graph.add_vertex();
  times.extend(destination, neg_inf_time);
  coordinates.extend(destination, instance.get_origin());

  for(const Request & request : instance.get_requests())
  {
    add_request_edges(request);
  }

  add_flight_edges();

  add_waiting_edges();
}

Solution::Path FundamentalPathGraph::solution_path_for(const Path& path) const
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

    if(edge.get_target() == destination ||
       edge.get_source() == origin)
    {
      continue;
    }

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

std::vector<Path> FundamentalPathGraph::paths_for(const Solution::Path& solution_path) const
{
  std::vector<Solution::Path> solution_paths;

  Solution::Path current_solution_path;

  for(const auto& step : solution_path.get_steps())
  {
    if(step.get_type() == OpType::BASE_WAITING)
    {
      assert(!step.get_request());

      if(!current_solution_path.get_steps().empty())
      {
        solution_paths.push_back(current_solution_path);

        current_solution_path = Solution::Path();
      }
    }
    else
    {
      current_solution_path.append(step);
    }
  }

  if(!current_solution_path.get_steps().empty())
  {
    solution_paths.push_back(current_solution_path);

    current_solution_path = Solution::Path();
  }

  std::vector<Path> paths;

  for(const auto& current_solution_path : solution_paths)
  {
    std::deque<std::shared_ptr<Request>> requests;

    std::deque<std::shared_ptr<Request>> all_requests;

    for(const auto& step : current_solution_path.get_steps())
    {
      auto request = step.get_request();

      if(request)
      {
        requests.push_back(request);
      }
    }

    assert(!requests.empty());

    all_requests = requests;

    Path path;

    {
      auto initial_request = requests.front();
      requests.pop_front();

      const Path& initial_path = get_request_paths()(*initial_request);
      Edge request_edge = get_request_edges()(*initial_request);

      bool found = false;

      for(const Edge& edge : initial_path.get_edges())
      {
        path.append(edge);

        if(edge == request_edge)
        {
          found = true;
          break;
        }
      }

      assert(found);
    }

    for(const auto& request : requests)
    {
      Edge request_edge = get_request_edges()(*request);

      Vertex current_vertex = path.get_target();

      bool found = false;

      for(const Edge& edge : graph.get_outgoing(current_vertex))
      {
        if(edge.get_target() == request_edge.get_source())
        {
          found = true;

          path.append(edge);

          break;
        }
      }

      assert(found);

      path.append(request_edge);
    }

    {
      Vertex current_vertex = path.get_target();

      bool found = false;

      for(const Edge& edge : graph.get_outgoing(current_vertex))
      {
        if(get_edge_types()(edge) == OpType::DESCENT)
        {
          path.append(edge);
          found = true;
          break;
        }
      }

      assert(found);
    }

    {
      Vertex current_vertex = path.get_target();

      bool found = false;

      for(const Edge& edge : graph.get_outgoing(current_vertex))
      {
        if(get_edge_types()(edge) == OpType::BASE_WAITING)
        {
          path.append(edge);
          found = true;
          break;
        }
      }

      assert(found);
    }

    {
      RequestPattern path_pattern(instance,
                                  get_edge_types(),
                                  get_edge_requests(),
                                  path);

      RequestPattern solution_path_pattern(instance);

      for(const auto& request : all_requests)
      {
        solution_path_pattern.insert(*request);
      }

      assert(path_pattern == solution_path_pattern);

    }

    assert(path.connects(get_origin(), get_destination()));

    paths.push_back(path);
  }

  return paths;
}

const std::vector<Vertex>& FundamentalPathGraph::get_topological_ordering() const
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

void FundamentalPathGraph::add_waiting_edges()
{
  auto base_duration = parameters.get_base_duration();

  for(const auto& pair : landing_vertices)
  {
    const DateTime landing_time = pair.first;
    const Vertex landing_vertex = pair.second;

    auto begin = takeoff_vertices.lower_bound(landing_time);

    auto end = takeoff_vertices.upper_bound(landing_time + base_duration);

    for(auto it = begin; it != end; ++it)
    {
      const DateTime takeoff_time = it->first;
      const Vertex takeoff_vertex = it->second;

      assert(landing_time <= takeoff_time);

      // corner case...
      if(takeoff_time == landing_time + base_duration)
      {
        continue;
      }

      assert(takeoff_time < landing_time + base_duration);

      add_edge(landing_vertex, takeoff_vertex, OpType::BASE_WAITING, Request());
    }
  }
}

Edge FundamentalPathGraph::add_edge(const Vertex& source,
                                    const Vertex& target,
                                    const OpType& type,
                                    const Request request)
{
  Edge edge = graph.add_edge(source, target);
  edge_types.extend(edge, type);
  edge_requests.extend(edge, request);
  return edge;
}

void FundamentalPathGraph::add_request_edges(const Request& request)
{
  const Point origin_point = instance.get_origin();

  const Units::Velocity flight_speed = parameters.get_flight_speed();

  Vertex request_source = graph.add_vertex();

  times.extend(request_source, request.get_time());
  coordinates.extend(request_source, request.get_origin());

  Vertex request_target = graph.add_vertex();

  DateTime finish_time = request.get_time() + parameters.get_refueling_duration();

  times.extend(request_target, finish_time);
  coordinates.extend(request_target, request.get_destination());

  Path request_path;

  request_edges(request) = add_edge(request_source, request_target, OpType::REFUELING, request);

  DateTime waiting_time;
  DateTime takeoff_time;

  {
    const Units::Length distance = std::max(origin_point.distance(request.get_origin()),
                                            parameters.get_takeoff().get_distance());

    const Units::Time climb_time = distance / flight_speed;

    takeoff_time = request.get_time() - climb_time;
    waiting_time = takeoff_time - parameters.get_base_duration();
  }

  const Vertex request_waiting = graph.add_vertex();

  times.extend(request_waiting, waiting_time);
  coordinates.extend(request_waiting, origin_point);

  const Vertex request_takeoff = graph.add_vertex();
  times.extend(request_takeoff, takeoff_time);
  coordinates.extend(request_takeoff, origin_point);

  takeoff_vertices.insert(std::make_pair(takeoff_time, request_takeoff));

  // takeoff waiting edge
  const Edge takeoff_edge = add_edge(origin, request_waiting, OpType::BASE_WAITING);

  request_path.append(takeoff_edge);

  // refueling edge
  {
    const Edge refueling_edge = add_edge(request_waiting, request_takeoff, OpType::BASE_REFUELING);

    request_path.append(refueling_edge);

    refueling_edges(request) = refueling_edge;
  }

  // takeoff edge
  {
    const Edge climb_edge = add_edge(request_takeoff, request_source, OpType::CLIMB);

    request_path.append(climb_edge);

    climb_edges(request) = climb_edge;
  }

  request_path.append(request_edges(request));

  const Vertex request_landing = graph.add_vertex();

  {
    const Units::Length distance = request.get_destination().distance(origin_point);
    const Units::Time descent_duration = distance / flight_speed;
    const DateTime landing_time = finish_time + descent_duration;

    times.extend(request_landing, landing_time);
    coordinates.extend(request_landing, instance.get_origin());

    times(destination) = std::max(times(destination), landing_time);

    landing_vertices.insert(std::make_pair(landing_time, request_landing));
  }

  // landing edge
  {
    const Edge descent_edge = add_edge(request_target, request_landing, OpType::DESCENT);

    request_path.append(descent_edge);

    descent_edges(request) = descent_edge;
  }

  // landing waiting edge
  const Edge landing_waiting_edge = add_edge(request_landing, destination, OpType::BASE_WAITING);

  request_path.append(landing_waiting_edge);

  request_paths(request) = request_path;
}

void FundamentalPathGraph::add_flight_edges()
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

      Edge current_edge = request_edges(current_request);
      Edge next_edge = request_edges(next_request);

      Edge flight_edge = graph.add_edge(current_edge.get_target(),
                                        next_edge.get_source());

      edge_types.extend(flight_edge, OpType::FLIGHT);
      edge_requests.extend(flight_edge, Request());
    }
  }
}

TimeSpan FundamentalPathGraph::get_timespan(const Path& path) const
{
  assert(path.connects(origin, destination));

  DateTime begin = times(path.get_edges().front().get_target());
  DateTime end = times(path.get_edges().back().get_source());

  return TimeSpan(begin, end);
}
