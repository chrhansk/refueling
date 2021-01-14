#include "iterative_covering_heuristic.hh"

#include "bounded_router/bounded_router.hh"

#include "log.hh"

#define NAME "initial_heuristic"

IterativeCoveringHeuristic::IterativeCoveringHeuristic(SchedulingProgram& program,
                                                       PathPricer& pricer)
  : scip::ObjHeur(program.getSCIP(),                 // SCIP
                  NAME,                              // name
                  "Computes greedy coverings",       // description
                  'i'   ,                            // display character
                  10000,                             // priority
                  0,                                 // frequency
                  0,                                 // frequency offset
                  0,                                 // max depth
                  SCIP_HEURTIMING_BEFORENODE,        // timing
                  FALSE),                            // subscip
  program(program),
  pricer(pricer),
  request_graph(program.get_request_graph()),
  graph(request_graph)
{}

SCIP_DECL_HEUREXEC(IterativeCoveringHeuristic::scip_exec)
{
  Log(info) << "Computing an iterative greedy cover";

  *result = SCIP_DIDNOTFIND;

  std::vector<Path> paths;

  RequestPattern uncovered = RequestPattern::all(request_graph.get_instance());

  BoundedRouter router(request_graph,
                       request_graph.get_topological_ordering(),
                       request_graph.get_parameters(),
                       request_graph.fuel_difference());

  NegRequestCount request_count(request_graph.get_edge_types());

  EdgeSet forbidden_edges(request_graph);

  const Instance& instance = request_graph.get_instance();

  const idx num_requests = instance.get_requests().size();

  auto get_score = [&] (const RequestPath& first) -> idx
    {
      return (first.pattern & uncovered).size();
    };

  auto score_compare = [&](const RequestPath& first, const RequestPath& second) -> bool
    {
      return get_score(first) < get_score(second);
    };

  auto add_path = [&](const RequestPath& request_path)
  {
    const Path& path = request_path.path;
    RequestPattern pattern(request_path.pattern);

    for(const Request& request : instance.get_requests())
    {
      if(!pattern.contains(request))
      {
        continue;
      }

      Edge request_edge = request_graph.get_request_edges()(request);

      forbidden_edges.insert(request_edge);

      for(const Edge& incoming : graph.get_incoming(request_edge.get_source()))
      {
        forbidden_edges.insert(incoming);
      }

      for(const Edge& outgoing : graph.get_outgoing(request_edge.get_source()))
      {
        forbidden_edges.insert(outgoing);
      }
    }

    paths.push_back(path);

    uncovered -= pattern;
  };

  while(!uncovered.empty())
  {
    if((int) paths.size() >= program.get_max_num_paths())
    {
      return SCIP_OKAY;
    }

    auto result = router.find_paths(request_count,
                                    forbidden_edges,
                                    request_graph.get_origin(),
                                    request_graph.get_destination());

    if(result.get_paths().empty())
    {
      return SCIP_OKAY;
    }

    std::vector<RequestPath> request_paths;

    for(const auto& path : result.get_paths())
    {
      request_paths.push_back(RequestPath{path, RequestPattern(request_graph.get_instance(),
                                                               request_graph.get_edge_types(),
                                                               request_graph.get_edge_requests(),
                                                               path)});
    }

    auto max_path = [&] () -> const RequestPath&
      {
        auto it = std::max_element(std::begin(request_paths),
                                   std::end(request_paths),
                                   score_compare);

        assert(it != std::end(request_paths));

        return *it;
      };

    const RequestPath& request_path = max_path();

    add_path(request_path);

    const idx num_covered = (num_requests - uncovered.size());

    Log(info) << "Covered "
              << num_covered
              << " of "
              << num_requests
              << " requests ("
              << 100*(((double) num_covered) / num_requests)
              << "%)";
  }

  Log(info) << "Found a solution with " << paths.size() << " paths";

  std::vector<std::shared_ptr<PathVariable>> variables;

  for(const Path& path : paths)
  {
    variables.push_back(pricer.add_path(path, false));
  }

  if(pricer.add_solution(variables, heur))
  {
    *result = SCIP_FOUNDSOL;
  }

  return SCIP_OKAY;
}
