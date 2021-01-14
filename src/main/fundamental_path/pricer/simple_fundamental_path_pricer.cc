#include "simple_fundamental_path_pricer.hh"

#include "log.hh"

#include "program/reduced_costs.hh"

SimpleFundamentalPathPricer::SimpleFundamentalPathPricer(FundamentalPathProgram& program)
  : FundamentalPathPricer(program),
    router(graph,
           fundamental_path_graph.get_topological_ordering(),
           fundamental_path_graph.get_parameters(),
           fundamental_path_graph.fuel_difference()),
    forbidden_path_router(graph,
                          fundamental_path_graph.get_topological_ordering(),
                          fundamental_path_graph.get_parameters(),
                          fundamental_path_graph.fuel_difference()),
    initiated(false)
{}

FundamentalPathPricer::PricingResult SimpleFundamentalPathPricer::perform_pricing(DualCostType cost_type)
{
  if(!initiated)
  {
    const RequestMap<Path>& request_paths = fundamental_path_graph.get_request_paths();

    std::vector<Path> paths;

    for(const Request& request : instance.get_requests())
    {
      paths.push_back(request_paths(request));
    }

    initiated = true;

    return PricingResult(paths);
  }

  auto special_paths = get_special_paths();

  FundamentalPathDualValues dual_values = program.get_dual_values(cost_type,
                                                                  special_paths.entire_paths);

  EdgeSet forbidden_edges = get_forbidden_edges(special_paths.required_paths);

  std::vector<Path> forbidden_paths;

  for(const auto& forbidden_path : special_paths.forbidden_paths)
  {
    forbidden_paths.push_back(forbidden_path->get_path());
  }

  if(cost_type == DualCostType::SIMPLE)
  {
    ReducedCosts reduced_costs(program.get_costs(), dual_values);

    BoundedRouter::Result result;

    if(forbidden_paths.empty())
    {
      result = router.find_paths(reduced_costs,
                                 forbidden_edges,
                                 fundamental_path_graph.get_origin(),
                                 fundamental_path_graph.get_destination());
    }
    else
    {
      result = forbidden_path_router.find_paths(reduced_costs,
                                                forbidden_edges,
                                                forbidden_paths,
                                                fundamental_path_graph.get_origin(),
                                                fundamental_path_graph.get_destination());
    }

    return PricingResult(result.get_paths());
  }
  else
  {
    assert(cost_type == DualCostType::FARKAS);

    FarkasCosts farkas_costs(dual_values);

    BoundedRouter::Result result;

    if(forbidden_paths.empty())
    {
      result = router.find_paths(farkas_costs,
                                 forbidden_edges,
                                 fundamental_path_graph.get_origin(),
                                 fundamental_path_graph.get_destination());
    }
    else
    {
      result = forbidden_path_router.find_paths(farkas_costs,
                                                forbidden_edges,
                                                forbidden_paths,
                                                fundamental_path_graph.get_origin(),
                                                fundamental_path_graph.get_destination());
    }



    return PricingResult(result.get_paths());
  }

}
