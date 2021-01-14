#include "simple_path_pricer.hh"

#include "log.hh"

SimplePathPricer::SimplePathPricer(SchedulingProgram& program,
                                   const FuelCostFunction<double>& costs,
                                   bool use_artificial_variables)
  : PathPricer(program, costs, use_artificial_variables),
    parameters(request_graph.get_parameters()),
    router(graph,
           request_graph.get_topological_ordering(),
           parameters,
           request_graph.fuel_difference()),
    forbidden_path_router(graph,
                          request_graph.get_topological_ordering(),
                          parameters,
                          request_graph.fuel_difference())
{
  const Instance& instance = request_graph.get_instance();

  for(const Request& request : instance.get_requests())
  {
    request_edges.push_back(request_graph.get_request_edges()(request));
  }
}

PathPricer::PricingResult SimplePathPricer::compute_simple_bound(double upper_bound,
                                                                 const BoundedRouter::Result& result) const
{
  if(result.get_paths().empty())
  {
    return PricingResult();
  }

  const num max_num_paths = program.get_max_num_paths();

  const double min_cost = result.get_min_cost();

  double lower_bound = upper_bound + max_num_paths * min_cost;


  Log(info) << "Lower bound: "
            << lower_bound
            << " based on min cost = "
            << min_cost
            << " and a maximum number of "
            << max_num_paths
            << " paths";


  return PricingResult(result.get_paths(), adjust_bound(lower_bound));
}

PathPricer::PricingResult SimplePathPricer::perform_pricing(DualCostType cost_type)
{
  SCIP* scip = program.getSCIP();

  const double upper_bound = SCIPgetLPObjval(scip);

  auto special_path_variables = get_special_path_variables();
  auto forbidden_edges = get_forbidden_edges(special_path_variables.required_paths);

  std::vector<Path> forbidden_paths;

  DualValues dual_values = program.get_dual_values(cost_type,
                                                   special_path_variables.entire_paths);

  for(const auto& forbidden_path_variable : special_path_variables.forbidden_paths)
  {
    forbidden_paths.push_back(forbidden_path_variable->get_path());
  }

  if(cost_type == DualCostType::FARKAS)
  {
    FarkasCosts farkas_costs(dual_values);

    BoundedRouter::Result result;

    if(forbidden_paths.empty())
    {
      result = router.find_paths(farkas_costs,
                                 forbidden_edges,
                                 request_graph.get_origin(),
                                 request_graph.get_destination());
    }
    else
    {
      result = forbidden_path_router.find_paths(farkas_costs,
                                                forbidden_edges,
                                                forbidden_paths,
                                                request_graph.get_origin(),
                                                request_graph.get_destination());
    }


    return PricingResult(result.get_paths());
  }
  else
  {
    assert(cost_type == DualCostType::SIMPLE);

    ReducedCosts reduced_costs(get_costs(), dual_values);

    BoundedRouter::Result result;

    if(forbidden_paths.empty())
    {
      result = router.find_paths(reduced_costs,
                                 forbidden_edges,
                                 request_graph.get_origin(),
                                 request_graph.get_destination());
    }
    else
    {
      result = forbidden_path_router.find_paths(reduced_costs,
                                                forbidden_edges,
                                                forbidden_paths,
                                                request_graph.get_origin(),
                                                request_graph.get_destination());
    }

    return compute_simple_bound(upper_bound, result);
  }
}
