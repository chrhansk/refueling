#include "fundamental_path_pricer.hh"

#include "program/scip_utils.hh"

#include "log.hh"

#define NAME "fundamental_path_pricer"

FundamentalPathPricer::FundamentalPathPricer(FundamentalPathProgram& program)
  : scip::ObjPricer(program.getSCIP(),
                    NAME,
                    "Prices fundamental paths",
                    1000,
                    true), // delay to make sure that reduced costs are nonnegative
    program(program),
    scip(program.getSCIP()),
    fundamental_path_graph(program.get_fundamental_path_graph()),
    graph(fundamental_path_graph),
    instance(fundamental_path_graph.get_instance()),
    edge_paths(fundamental_path_graph, std::vector<std::shared_ptr<FundamentalPath>>{}),
    covering_constraints(program.get_covering_constraints()),
    request_edges(fundamental_path_graph.get_request_edges())
{

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_CONS* covering_constraint = covering_constraints(edge);

    if(covering_constraint)
    {
      SCIP_CALL_EXC(SCIPcaptureCons(scip, covering_constraint));
    }
  }

}

std::string FundamentalPathPricer::get_name() const
{
  return NAME;
}

FundamentalPathPricer::SpecialPaths
FundamentalPathPricer::get_special_paths() const
{
  FundamentalPathPricer::SpecialPaths special_paths;

  for(const auto& pair : get_paths())
  {
    const auto& path = pair.second;

    SCIP_VAR* variable = path->get_trip_variable();

    if(SCIPisFeasEQ(scip, SCIPvarGetUbLocal(variable), 0.))
    {
      special_paths.forbidden_paths.push_back(path);
    }
    else if(SCIPisFeasEQ(scip, SCIPvarGetLbLocal(variable), 1.))
    {
      special_paths.required_paths.push_back(path);
    }
    else
    {
      if(SCIPisFeasEQ(scip, SCIPgetSolVal(scip, nullptr, variable), 1.))
      {
        special_paths.entire_paths.push_back(path);
      }
    }
  }

  Log(debug) << "There are "
             << special_paths.forbidden_paths.size()
             << " forbidden and "
             << special_paths.required_paths.size()
             << " required paths.";

  return special_paths;
}

const FundamentalPath& FundamentalPathPricer::add_path(const Path& path,
                                                       const TimeSpan& timespan)
{
  SCIP_VAR* trip_variable;

  int index = paths.size();

  assert(!has_path(path));

  {
    std::ostringstream namebuf;

    namebuf << "y_" << index;

    const double path_costs = program.get_costs().cost(path,
                                                       fundamental_path_graph.fuel_difference());

    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &trip_variable,
                                namebuf.str().c_str(),
                                0.0,                  // lower
                                1.0,                  // upper
                                path_costs,           // cost
                                SCIP_VARTYPE_BINARY,  // vartype
                                TRUE,                 // initial
                                TRUE,                 // removable
                                NULL, NULL, NULL, NULL, NULL));

    SCIP_CALL_EXC(SCIPchgVarBranchPriority(scip, trip_variable, 0));

    SCIP_CALL_EXC(SCIPaddVar(scip, trip_variable));

  }

  RequestPattern request_pattern(fundamental_path_graph.get_instance(),
                                 fundamental_path_graph.get_edge_types(),
                                 fundamental_path_graph.get_edge_requests(),
                                 path);

  if(debugging_enabled())
  {
    std::ostringstream namebuf;

    namebuf << "[";

    for(const Request& request : instance.get_requests())
    {
      if(request_pattern.contains(request))
      {
        namebuf << request.get_index() << ", ";
      }
    }

    namebuf << "]";

    Log(debug) << "Adding pattern " << namebuf.str();
  }

  for(const Request& request : instance.get_requests())
  {
    if(request_pattern.contains(request))
    {
      SCIP_CONS* covering_constraint = covering_constraints(request_edges(request));

      assert(covering_constraint);

      SCIP_CALL_EXC(SCIPaddCoefLinear(scip, covering_constraint, trip_variable, 1.));
    }
  }

  for(const Edge& edge : path.get_edges())
  {
    for(const auto& clique_constraint : program.get_clique_constraints())
    {
      if(!clique_constraint.get_edges().contains(edge))
      {
        continue;
      }

      SCIP_CONS* constraint = clique_constraint.get_constraint();

      SCIP_CALL_EXC(SCIPaddCoefLinear(scip, constraint, trip_variable, 1.));
    }
  }

  std::shared_ptr<FundamentalPath> fundamental_path = std::make_shared<FundamentalPath>(path,
                                                                                        index,
                                                                                        request_pattern,
                                                                                        trip_variable,
                                                                                        timespan);

  paths.insert({path, fundamental_path});

  for(const Edge& edge : path.get_edges())
  {
    edge_paths(edge).push_back(fundamental_path);
  }

  program.added_path(*fundamental_path);

  return *fundamental_path;
}

bool FundamentalPathPricer::has_path(const Path& path) const
{
  return paths.find(path) != paths.end();
}

void FundamentalPathPricer::add_dual_values(EdgeMap<double>& dual_values,
                                            DualCostType cost_type) const
{
  for(const Request& request : instance.get_requests())
  {
    const Edge& request_edge = request_edges(request);

    SCIP_CONS* constraint;

    SCIP_CALL_EXC(SCIPgetTransformedCons(scip,
                                         covering_constraints(request_edge),
                                         &constraint));

    assert(constraint);

    double dual_value = cons_dual_value(scip, constraint, cost_type);

    dual_values(request_edge) += dual_value;
  }

  for(const auto& clique_constraint : program.get_clique_constraints())
  {
    SCIP_CONS* constraint;

    SCIP_CALL_EXC(SCIPgetTransformedCons(scip,
                                         clique_constraint.get_constraint(),
                                         &constraint));

    double dual_value = cons_dual_value(scip, constraint, cost_type);

    if(cmp::zero(dual_value))
    {
      continue;
    }

    for(const Edge& edge : graph.get_edges())
    {
      if(clique_constraint.get_edges().contains(edge))
      {
        dual_values(edge) += dual_value;
      }
    }
  }
}

SCIP_SOL* FundamentalPathPricer::create_solution(const std::vector<Path>& paths,
                                                 SCIP_HEUR* heur)
{
  SCIP_SOL* sol;
  SCIP_CALL_EXC(SCIPcreateSol(scip, &sol, heur));

  for(const Path& path : paths)
  {
    FundamentalPath fundamental_path = add_path(path, fundamental_path_graph.get_timespan(path));

    SCIP_CALL_EXC(SCIPsetSolVal(scip, sol, fundamental_path.get_trip_variable(), 1.));
  }

  return sol;
}

SCIP_DECL_PRICERINIT(FundamentalPathPricer::scip_init)
{
  for(const Request& request : instance.get_requests())
  {
    const Edge& edge = request_edges(request);

    SCIP_CONS* covering_constraint = covering_constraints(edge);

    if(covering_constraint)
    {
      SCIP_CONS* transformed_constraint;

      SCIP_CALL_EXC(SCIPgetTransformedCons(scip,
                                           covering_constraint,
                                           &transformed_constraint));

      SCIP_CALL_EXC(SCIPcaptureCons(scip, transformed_constraint));

      SCIP_CALL_EXC(SCIPreleaseCons(scip, &covering_constraint));

      covering_constraints(edge) = transformed_constraint;
    }
  }

  return SCIP_OKAY;
}

SCIP_DECL_PRICEREXIT(FundamentalPathPricer::scip_exit)
{
  for(auto& pair : paths)
  {
    auto& fundamental_path = pair.second;

    SCIP_VAR* var = fundamental_path->get_trip_variable();

    assert(var);

    SCIP_CALL_ASSERT(SCIPreleaseVar(scip, &var));

    fundamental_path->set_trip_variable(nullptr);
  }

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_CONS* covering_constraint = covering_constraints(edge);

    if(covering_constraint)
    {
      SCIP_CALL_EXC(SCIPreleaseCons(scip, &covering_constraint));

      covering_constraints(edge) = nullptr;
    }
  }

  return SCIP_OKAY;
}

EdgeSet FundamentalPathPricer::get_forbidden_edges(const std::vector<std::shared_ptr<FundamentalPath>>& required_fundamental_paths) const
{
  EdgeSet forbidden_edges = program.get_forbidden_edges();

  const auto& edge_types = fundamental_path_graph.get_edge_types();

  for(const auto& required_fundamental_path : required_fundamental_paths)
  {
    const Path& required_path = required_fundamental_path->get_path();

    for(const Edge& edge : required_path.get_edges())
    {
      if(edge_types(edge) != OpType::BASE_WAITING &&
         edge_types(edge) != OpType::BASE_REFUELING)
      {
        forbidden_edges.insert(edge);
      }
    }
  }

  return forbidden_edges;
}

SCIP_DECL_PRICERFARKAS(FundamentalPathPricer::scip_farkas)
{
  //check_branching_variables();

  *result = SCIP_SUCCESS;

  PricingResult pricing_result = perform_pricing(DualCostType::FARKAS);

  if(!pricing_result.get_paths().empty())
  {
    Log(info) << "Adding " << pricing_result.get_paths().size() << " fundamental paths";
  }

  for(const Path& path : pricing_result.get_paths())
  {
    add_path(path, fundamental_path_graph.get_timespan(path));
  }

  Log(info) << "Number of fundamental paths: " << paths.size();

  return SCIP_OKAY;
}

SCIP_DECL_PRICERREDCOST(FundamentalPathPricer::scip_redcost)
{
  //check_branching_variables();

  *result = SCIP_SUCCESS;

  PricingResult pricing_result = perform_pricing(DualCostType::SIMPLE);

  if(!pricing_result.get_paths().empty())
  {
    Log(info) << "Adding " << pricing_result.get_paths().size() << " fundamental path";
  }

  for(const Path& path : pricing_result.get_paths())
  {
    add_path(path, fundamental_path_graph.get_timespan(path));
  }

  Log(info) << "Number of fundamental path: " << paths.size();

  return SCIP_OKAY;
}
