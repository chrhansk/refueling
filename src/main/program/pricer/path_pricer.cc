#include "path_pricer.hh"

#include "log.hh"

#include "edge_fuel_difference.hh"
#include "program/scip_utils.hh"
#include "program/reduced_costs.hh"

#define NAME "path_pricer"

PathPricer::PathPricer(SchedulingProgram& program,
                       const FuelCostFunction<double>& costs,
                       bool use_artificial_variables)
  : scip::ObjPricer(program.getSCIP(),
                    NAME,
                    "Prices paths",
                    1000,
                    TRUE), // delay to make sure that reduced costs are nonnegative
    request_graph(program.get_request_graph()),
    graph(request_graph),
    instance(program.get_instance()),
    program(program),
    scip(program.getSCIP()),
    edge_variables(request_graph,
                   std::vector<std::shared_ptr<PathVariable>>{}),
    covering_constraints(program.get_covering_constraints()),
    path_constraint(program.get_path_constraint()),
    use_artificial_variables(use_artificial_variables),
    initiated(false),
    costs(costs)
{

}

SCIP_DECL_PRICERINIT(PathPricer::scip_init)
{
  {
    SCIP_CONS* cons = path_constraint;

    if(cons)
    {
      SCIP_CALL_EXC(SCIPgetTransformedCons(scip,
                                           path_constraint,
                                           &cons));

      path_constraint = cons;
    }
  }

  // TODO: port this to a request edge map
  for(const Vertex& vertex : graph.get_vertices())
  {
    SCIP_CONS* cons = covering_constraints(vertex);

    if(cons)
    {
      SCIP_CALL_EXC(SCIPgetTransformedCons(scip,
                                           covering_constraints(vertex),
                                           &cons));
    }

    covering_constraints(vertex) = cons;
  }

  return SCIP_OKAY;
}

double PathPricer::adjust_bound(double bound) const
{
  if(get_costs().nonnegative())
  {
    bound = std::max(bound, 0.);
  }

  if(get_costs().integral())
  {
    bound = cmp::ceil(bound);
  }

  return bound;
}

PathPricer::SpecialPathVariables
PathPricer::get_special_path_variables() const
{
  SpecialPathVariables special_path_variables;

  for(const auto& pair : path_variables)
  {
    auto path_variable = pair.second;
    SCIP_VAR* variable = path_variable->get_var();

    if(SCIPisFeasEQ(scip, SCIPvarGetUbLocal(variable), 0.))
    {
      special_path_variables.forbidden_paths.push_back(path_variable);
    }
    else if(SCIPisFeasEQ(scip, SCIPvarGetLbLocal(variable), 1.))
    {
      special_path_variables.required_paths.push_back(path_variable);
    }
    else
    {
      if(SCIPisFeasEQ(scip, SCIPgetSolVal(scip, nullptr, variable), 1.))
      {
        special_path_variables.entire_paths.push_back(path_variable);
      }
    }
  }

  Log(debug) << "There are "
             << special_path_variables.forbidden_paths.size()
             << " forbidden and "
             << special_path_variables.required_paths.size()
             << " required paths.";

  return special_path_variables;
}

SCIP_DECL_PRICEREXIT(PathPricer::scip_exit)
{
  for(const auto& pair : path_variables)
  {
    SCIP_VAR* var = pair.second->get_var();

    SCIP_CALL_EXC(SCIPreleaseVar(scip, &var));
  }

  path_variables.clear();

  for(const auto& artifical_variable : artificial_variables)
  {
    SCIP_VAR* var = artifical_variable.get_var();
    SCIP_CALL_EXC(SCIPreleaseVar(scip, &var));
  }

  artificial_variables.clear();

  return SCIP_OKAY;
}

SCIP_DECL_PRICERFARKAS(PathPricer::scip_farkas)
{
  *result = SCIP_SUCCESS;

  if(!initiated)
  {
    if(use_artificial_variables)
    {
      for(const Request& request : instance.get_requests())
      {
        add_artificial_variable(request);
      }
    }

    for(const Request& request : instance.get_requests())
    {
      add_path(request_graph.get_request_path(request));
    }

    initiated = true;

    return SCIP_OKAY;
  }

  PricingResult pricing_result = perform_pricing(DualCostType::FARKAS);

  const bool empty = pricing_result.get_paths().empty();

  if(!empty)
  {
    Log(info) << "Adding " << pricing_result.get_paths().size() << " paths";

    for(const Path& path : pricing_result.get_paths())
    {
      assert(!has_path(path));
      add_path(path);
    }
  }

  return SCIP_OKAY;
}

SCIP_DECL_PRICERREDCOST(PathPricer::scip_redcost)
{
  PricingResult pricing_result = perform_pricing(DualCostType::SIMPLE);

  const bool empty = pricing_result.get_paths().empty();

  if(!empty)
  {
    const auto& paths = pricing_result.get_paths();

    if(debugging_enabled())
    {
      for(const auto& path : paths)
      {
        if(has_path(path))
        {
          auto it = path_variables.find(path);
          SCIP_VAR* var = it->second->get_var();

          double redcost = SCIPgetVarRedcost(scip, var);

          Log(info) << "Reduced cost of existing path: " << redcost;

        }

        assert(!has_path(path));
      }
    }

    Log(info) << "Adding " << paths.size() << " paths";

    for(const Path& path : pricing_result.get_paths())
    {
      // Note: The router can sometimes return multiple
      // copies of the same path.
      if(!has_path(path))
      {
        add_path(path);
      }
    }
  }

  if(pricing_result.get_lower_bound() != -inf)
  {
    *lowerbound = adjust_bound(pricing_result.get_lower_bound());
  }

  *result = SCIP_SUCCESS;

  return SCIP_OKAY;
}

bool PathPricer::has_path(const Path& path) const
{
  auto it = path_variables.find(path);

  return (it != std::end(path_variables));
}

std::shared_ptr<PathVariable> PathPricer::add_path(const Path& path,
                                                   bool priced)
{
  assert(path.is_simple());
  assert(path.connects(request_graph.get_origin(),
                       request_graph.get_destination()));

  auto it = path_variables.find(path);

  if(it != std::end(path_variables))
  {
    return it->second;
  }

  EdgeFuelDifference fuel_difference = request_graph.fuel_difference();

  double cost = get_costs().cost(path, fuel_difference);

  assert(cost >= 0);

  SCIP_VAR* var;

  std::ostringstream namebuf;

  namebuf << "y_" << path_variables.size();

  SCIP_CALL_EXC(SCIPcreateVar(scip,
                              &var,
                              namebuf.str().c_str(),
                              0.0,
                              1.0,
                              cost,
                              SCIP_VARTYPE_BINARY,
                              TRUE,   // initial
                              TRUE,   // removable
                              NULL, NULL, NULL, NULL, NULL));

  SCIP_CALL_EXC(SCIPchgVarBranchPriority(scip, var, 0));

  for(const Edge& edge : path.get_edges())
  {
    SCIP_CONS* covering_constraint = covering_constraints(edge.get_source());

    if(covering_constraint)
    {
      SCIP_CALL_EXC(SCIPaddCoefLinear(scip, covering_constraint, var, 1.0));
    }
  }

  if(path_constraint)
  {
    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, path_constraint, var, 1.0));
  }

  if(priced)
  {
    SCIP_CALL_EXC(SCIPaddPricedVar(scip, var, 1.));
  }
  else
  {
    SCIP_CALL_EXC(SCIPaddVar(scip, var));
  }

  std::shared_ptr<PathVariable> variable = std::make_shared<PathVariable>(var,
                                                                          path_variables.size(),
                                                                          path,
                                                                          request_graph);

  path_variables.insert({path, variable});


  EdgeMap<std::vector<std::shared_ptr<PathVariable>>> variables(request_graph,
                                                                std::vector<std::shared_ptr<PathVariable>>{});

  for(const Edge& edge : variable->get_path().get_edges())
  {
    edge_variables(edge).push_back(variable);
  }


  program.added_path_variable(*variable);

  return variable;
}

const EdgeFunc<std::vector<std::shared_ptr<PathVariable>>>& PathPricer::edge_path_variables() const
{
  return edge_variables;
}

EdgeMap<double> PathPricer::get_combined_values(SCIP_SOL* solution) const
{
  EdgeMap<double> values(request_graph, 0.);

  for(const auto& pair: get_path_variables())
  {
    const auto& path_variable = pair.second;

    const double value = SCIPgetSolVal(scip, solution, path_variable->get_var());

    for(const Edge& edge : path_variable->get_path().get_edges())
    {
      values(edge) += value;
    }
  }

  return values;
}

std::shared_ptr<PathVariable> PathPricer::get_path_variable(const Path& path) const
{
  auto it = path_variables.find(path);

  assert(it != std::end(path_variables));

  return it->second;
}

bool PathPricer::add_solution(const std::vector<std::shared_ptr<PathVariable>>& sol,
                              SCIP_HEUR* heur)
{
  SCIP_SOL* solution;
  SCIP_CALL_EXC(SCIPcreateSol(scip, &solution, heur));

  for(const auto& path_variable : sol)
  {
    SCIP_CALL_EXC(SCIPsetSolVal(scip, solution, path_variable->get_var(), 1));
  }

  SCIP_Bool stored;

  SCIP_CALL_EXC(SCIPaddSolFree(scip, &solution, &stored));

  if(stored)
  {
    Log(info) << "Solution was stored";
  }
  else
  {
    Log(info) << "Solution was not stored";
  }

  return stored;
}

void PathPricer::add_artificial_variable(const Request& request)
{
  SCIP_VAR* var;

  std::ostringstream namebuf;

  namebuf << "z_" << request.get_index();

  const double artifical_cost = 1e7;

  SCIP_CALL_EXC(SCIPcreateVar(scip,
                              &var,
                              namebuf.str().c_str(),
                              0.0,
                              SCIPinfinity(scip),
                              artifical_cost,
                              SCIP_VARTYPE_CONTINUOUS,
                              TRUE,   // initial
                              TRUE,   // removable
                              NULL, NULL, NULL, NULL, NULL));

  SCIP_CALL_EXC(SCIPchgVarBranchPriority(scip, var, 0));

  Edge request_edge = request_graph.get_request_edges()(request);

  SCIP_CONS* covering_constraint = covering_constraints(request_edge.get_source());

  assert(covering_constraint);

  SCIP_CALL_EXC(SCIPaddCoefLinear(scip, covering_constraint, var, 1.0));

  SCIP_CALL_EXC(SCIPaddVar(scip, var));

  artificial_variables.push_back(ArtificialVariable(var, request));
}

std::string PathPricer::get_name()
{
  return NAME;
}

EdgeSet
PathPricer::get_forbidden_edges(const std::vector<std::shared_ptr<PathVariable>>& required_path_variables) const
{
  EdgeSet forbidden_edges = program.get_forbidden_edges();

  const auto& edge_types = request_graph.get_edge_types();

  for(const auto& required_path_variable : required_path_variables)
  {

    const Path& required_path = required_path_variable->get_path();

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
