#include "greedy_covering.hh"

#include "log.hh"

void GreedyCovering::add_path(const Path& path,
                              double cost)
{
  request_paths.push_back(RequestPath{path, cost, RequestPattern(request_graph.get_instance(),
                                                                 request_graph.get_edge_types(),
                                                                 request_graph.get_edge_requests(),
                                                                 path)});
}

void GreedyCovering::add_path(const Path& path,
                              double cost,
                              const RequestPattern& pattern)
{
  request_paths.push_back(RequestPath{path, cost, pattern});
}

GreedyCovering::Result GreedyCovering::compute(idx max_num_paths)
{
  Log(info) << "Computing a greedy cover using "
            << request_paths.size()
            << " paths";

  if(request_paths.empty())
  {
    return Result::empty();
  }

  std::vector<Path> paths;

  RequestPattern uncovered = RequestPattern::all(request_graph.get_instance());

  double total_cost = 0;

  while(!uncovered.empty())
  {
    if(paths.size() >= max_num_paths)
    {
      Log(debug) << "Exceeded number of paths";
      return Result::empty();
    }

    auto it = std::min_element(std::begin(request_paths),
                               std::end(request_paths),
                               [&](const RequestPath& first, const RequestPath& second) -> bool
                               {
                                 double first_score = first.cost / (uncovered & first.pattern).size();
                                 double second_score = second.cost / (uncovered & second.pattern).size();

                                 return first_score < second_score;
                               });



    assert(it != std::end(request_paths));

    const RequestPath& request_path = *it;

    //Log(debug) << "Path covers " << (uncovered & request_path.pattern).size() << " new requests";

    uncovered -= request_path.pattern;
    paths.push_back(request_path.path);
    total_cost += request_path.cost;
  }

  Log(info) << "Found a cover using " << paths.size()
            << " paths with a cost of "
            << total_cost;

  return Result(paths, total_cost);
}
