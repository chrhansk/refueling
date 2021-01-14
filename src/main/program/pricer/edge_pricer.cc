#include "edge_pricer.hh"

#include "log.hh"

#include "program/path_variable.hh"

#define NAME "edge_pricer"

EdgePricer::EdgePricer(SchedulingProgram& program)
  : scip::ObjPricer(program.getSCIP(),
                    NAME,
                    "Prices edge variables",
                    500,
                    TRUE), // delay to make sure that reduced costs are nonnegative
    program(program),
    scip(program.getSCIP()),
    request_graph(program.get_request_graph()),
    graph(request_graph),
    instance(request_graph.get_instance()),
    linking_constraints(request_graph, nullptr),
    compound_variables(request_graph, nullptr)
{

}

SCIP_DECL_PRICEREXIT(EdgePricer::scip_init)
{
  for(const Edge& edge : graph.get_edges())
  {
    SCIP_CONS* linking_constraint = linking_constraints(edge);

    if(linking_constraint)
    {
      SCIP_CONS* transformed_constraint;

      SCIP_CALL_EXC(SCIPgetTransformedCons(scip, linking_constraint, &transformed_constraint));

      SCIP_CALL_EXC(SCIPcaptureCons(scip, transformed_constraint));

      SCIP_CALL_EXC(SCIPreleaseCons(scip, &linking_constraint));

      linking_constraints(edge) = transformed_constraint;
    }
  }

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_VAR* compound_variable = compound_variables(edge);

    if(compound_variable)
    {
      SCIP_VAR* transformed_variable;

      SCIP_CALL_EXC(SCIPgetTransformedVar(scip, compound_variable, &transformed_variable));

      SCIP_CALL_EXC(SCIPcaptureVar(scip, transformed_variable));

      SCIP_CALL_EXC(SCIPreleaseVar(scip, &compound_variable));

      compound_variables(edge) = transformed_variable;
    }
  }

  return SCIP_OKAY;
}

SCIP_DECL_PRICEREXIT(EdgePricer::scip_exit)
{
  for(const Edge& edge : graph.get_edges())
  {
    SCIP_CONS* linking_constraint = linking_constraints(edge);

    if(linking_constraint)
    {
      SCIP_CALL_EXC(SCIPreleaseCons(scip, &linking_constraint));
      linking_constraints(edge) = nullptr;
    }
  }

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_VAR* compound_variable = compound_variables(edge);

    if(compound_variable)
    {
      SCIP_CALL_EXC(SCIPreleaseVar(scip, &compound_variable));
      compound_variables(edge) = nullptr;
    }
  }

  return SCIP_OKAY;
}

SCIP_DECL_PRICERREDCOST(EdgePricer::scip_redcost)
{
  *result = SCIP_SUCCESS;

  EdgeMap<double> flow = program.total_flow();

  idx num_added_candidates = 0;

  for(const Request& request : instance.get_requests())
  {
    bool has_fractional = false;
    bool has_branching_candidate = false;

    Edge request_edge = request_graph.get_request_edges()(request);
    Vertex source = request_edge.get_source();

    Edge branching_candidate;

    for(const Edge& incoming : graph.get_incoming(source))
    {
      double flow_value = flow(incoming);

      if(SCIPisFeasIntegral(scip, flow_value))
      {
        continue;
      }

      if(compound_variables(incoming))
      {
        has_branching_candidate = true;
      }
      else
      {
        has_fractional = true;
        branching_candidate = incoming;
      }
    }

    if(has_fractional && !has_branching_candidate)
    {
      create_branching_candidate(branching_candidate);
      ++num_added_candidates;
    }

  }

  if(num_added_candidates)
  {
    Log(info) << "Created " << num_added_candidates << " branching candidates";
  }

  return SCIP_OKAY;
}

void EdgePricer::create_branching_candidate(const Edge& edge)
{
  SCIP_VAR* compound_variable;

  assert(!compound_variables(edge));
  assert(!linking_constraints(edge));

  {
    std::ostringstream namebuf;

    namebuf << "z_" << edge.get_source() << "_" << edge.get_target();


    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &compound_variable,
                                namebuf.str().c_str(),
                                0.0,    // lower
                                1.0,    // upper
                                0.0,    // cost
                                SCIP_VARTYPE_BINARY,
                                TRUE,   // initial
                                TRUE,   // removable
                                NULL, NULL, NULL, NULL, NULL));

    SCIP_CALL_EXC(SCIPchgVarBranchPriority(scip, compound_variable, 1));

    SCIP_CALL_EXC(SCIPaddVar(scip, compound_variable));

    compound_variables(edge) = compound_variable;

    edge_variables.push_back(EdgeVariable{edge, compound_variable});
  }

  {
    std::ostringstream namebuf;

    namebuf << "linking_" << edge.get_source() << "_" << edge.get_target();
    SCIP_CONS* linking_constraint;

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &linking_constraint,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       0,                   // lhs
                                       0,                   // rhs
                                       TRUE,                // initial
                                       TRUE,                // separate
                                       TRUE,                // enforce
                                       TRUE,                // check
                                       TRUE,                // propagate
                                       FALSE,               // local
                                       TRUE,                // modifiable
                                       FALSE,               // dy1namic
                                       FALSE,               // removable
                                       FALSE));             // sticking at node

    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, linking_constraint, compound_variable, -1.));

    for(const auto& variable : program.edge_path_variables()(edge))
    {
      SCIP_CALL_EXC(SCIPaddCoefLinear(scip, linking_constraint, variable->get_var(), 1.));
    }

    SCIP_CALL_EXC(SCIPaddCons(scip, linking_constraint));

    linking_constraints(edge) = linking_constraint;
  }
}

EdgeSet EdgePricer::get_forbidden_edges() const
{
  EdgeSet forbidden_edges(request_graph);

  idx num_forbidden = 0, num_required = 0;

  for(const EdgeVariable& edge_variable : edge_variables)
  {
    if(cmp::le(SCIPvarGetUbLocal(edge_variable.variable), 0.5))
    {
      forbidden_edges.insert(edge_variable.edge);
      ++num_forbidden;
    }
    else if(cmp::ge(SCIPvarGetLbLocal(edge_variable.variable), 0.5))
    {
      ++num_required;
    }
  }

  Log(debug) << "There are " << num_forbidden
             << " forbidden, " << num_required
             << " required edges, depth: "
             << SCIPgetDepth(scip);

  return forbidden_edges;
}

void EdgePricer::added_path_variable(const PathVariable& variable)
{
  for(const Edge& edge : variable.get_path().get_edges())
  {
    SCIP_CONS* linking_constraint = linking_constraints(edge);

    if(!linking_constraint)
    {
      continue;
    }

    SCIP_CALL_EXC(SCIPaddCoefLinear(scip, linking_constraint, variable.get_var(), 1.));

  }
}

void EdgePricer::add_dual_values(EdgeMap<double>& dual_values,
                                 DualCostType cost_type) const
{
  for(const EdgeVariable& edge_variable : edge_variables)
  {
    Edge edge = edge_variable.edge;

    SCIP_CONS* linking_constraint = linking_constraints(edge);

    assert(SCIPconsIsTransformed(linking_constraint));

    const double dual_value = cons_dual_value(scip, linking_constraint, cost_type);

    dual_values(edge) += dual_value;
  }
}

void EdgePricer::set_compound_solution_values(const std::vector<Path>& paths,
                                                    SCIP_SOL* sol)
{
  for(const Path& path : paths)
  {
    for(const Edge& edge : path.get_edges())
    {
      SCIP_VAR* compound_variable = compound_variables(edge);

      if(compound_variable)
      {
        SCIP_CALL_EXC(SCIPsetSolVal(scip, sol, compound_variable, 1.));
      }

    }
  }
}

std::string EdgePricer::get_name()
{
  return NAME;
}


bool EdgePricer::has_fixed_edges() const
{
  for(const EdgeVariable& edge_variable : edge_variables)
  {
    if(cmp::le(SCIPvarGetUbLocal(edge_variable.variable), 0.5))
    {
      return true;
    }
    else if(cmp::ge(SCIPvarGetLbLocal(edge_variable.variable), 0.5))
    {
      return true;
    }
  }

  return false;
}
