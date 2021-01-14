#include "capacity_constraint_handler.hh"

#include "log.hh"
#include "scip_utils.hh"
#include "path_variable.hh"

#define NAME "capacity_constraint"
#define DESC "Separates capacity constraints"

#define SEPA_PRIO 100000
#define ENFO_PRIO 100000
#define CHECK_PRIO -1000 // negative priority: only chech integral solutions

#define SEPA_FREQ 1
#define PROP_FREQ 1
#define EAGER_FREQ -1
#define MAX_PREROUNDS 0

#define DELAY_SEPA FALSE
#define DELAY_PROP TRUE
#define NEEDS_CONS TRUE

#define PRESOL_TIMING SCIP_PRESOLTIMING_FAST
#define PROP_TIMING SCIP_PROPTIMING_AFTERLPLOOP

CapacityConstraintHandler::CapacityConstraintHandler(SchedulingProgram& program)
  : scip::ObjConshdlr(program.getSCIP(),
                      NAME,
                      DESC,
                      SEPA_PRIO,
                      ENFO_PRIO,
                      CHECK_PRIO,
                      SEPA_FREQ,
                      PROP_FREQ,
                      EAGER_FREQ,
                      MAX_PREROUNDS,
                      DELAY_SEPA,
                      DELAY_PROP,
                      NEEDS_CONS,
                      PROP_TIMING,
                      PRESOL_TIMING),
    program(program),
    scip(program.getSCIP()),
    request_graph(program.get_request_graph()),
    graph(program.get_request_graph()),
    capacity_constraints(graph, nullptr)
{

}

std::string CapacityConstraintHandler::get_name() const
{
  return std::string(NAME);
}

void CapacityConstraintHandler::add_dual_values(EdgeMap<double>& dual_values,
                                                DualCostType cost_type) const
{
  for(const auto& edge : graph.get_edges())
  {
    dual_values(edge) += row_dual_value(capacity_constraints(edge), cost_type);
  }
}

void CapacityConstraintHandler::added_path_variable(const PathVariable& path_variable)
{
  for(const auto& edge : path_variable.get_path().get_edges())
  {
    SCIP_ROW* capacity_constraint = capacity_constraints(edge);

    if(capacity_constraint)
    {
      SCIP_CALL_EXC(SCIPaddVarToRow(scip,
                                    capacity_constraint,
                                    path_variable.get_var(),
                                    1.));
    }
  }
}

void CapacityConstraintHandler::create_capacity_constraint(SCIP_CONSHDLR* conshdlr,
                                                           const Edge& edge)
{
  SCIP_ROW* row;

  assert(!capacity_constraints(edge));

  std::ostringstream namebuf;

  const auto& variables = program.edge_path_variables();

  namebuf << "capacity_" << edge.get_source() << "_" << edge.get_target();

  SCIP_CALL_EXC(SCIPcreateEmptyRowCons(scip,
                                       &row,
                                       conshdlr,
                                       namebuf.str().c_str(),
                                       -SCIPinfinity(scip),      // lower
                                       1.,                       // upper
                                       FALSE,                    // local
                                       TRUE,                     // modifiable
                                       FALSE));                  // removable

  for(const auto& variable : variables(edge))
  {
    SCIP_CALL_EXC(SCIPaddVarToRow(scip, row, variable->get_var(), 1.));
  }

  SCIP_Bool infeasible;
  SCIP_CALL_EXC(SCIPaddRow(scip, row, TRUE, &infeasible));

  capacity_constraints(edge) = row;
}

bool CapacityConstraintHandler::separate(SCIP_CONSHDLR* conshdlr,
                                         SCIP_SOL* solution)
{
  Log(info) << "Separating capacity cuts";

  const auto& variables = program.edge_path_variables();

  bool separated = false;

  idx num_cuts = 0;

  for(const Edge& edge : graph.get_edges())
  {
    const OpType edge_type = request_graph.get_edge_types()(edge);

    if(edge_type != OpType::CLIMB &&
       edge_type != OpType::DESCENT &&
       edge_type != OpType::FLIGHT)
    {
      continue;
    }

    if(capacity_constraints(edge))
    {
      continue;
    }

    double value = 0.;

    for(const auto& variable : variables(edge))
    {
      value += SCIPgetSolVal(scip, solution, variable->get_var());
    }

    if(SCIPisFeasLE(scip, value, 1.))
    {
      continue;
    }

    /*
    if(capacity_constraints(edge))
    {
      SCIP_CALL_EXC(SCIPprintRow(scip, capacity_constraints(edge), stdout));
    }
    */


    //assert(!capacity_constraints(edge));

    create_capacity_constraint(conshdlr, edge);

    separated = true;

    ++num_cuts;
  }

  if(num_cuts)
  {
    Log(info) << "Separated " << num_cuts << " capacity cuts";
  }
  else
  {
    Log(info) << "Could not find any violated capacity cuts";
  }

  return separated;
}

bool CapacityConstraintHandler::is_violated(SCIP_SOL* solution)
{
  const auto& variables = program.edge_path_variables();

  for(const Edge& edge : graph.get_edges())
  {
    const OpType edge_type = request_graph.get_edge_types()(edge);

    if(edge_type != OpType::CLIMB &&
       edge_type != OpType::DESCENT &&
       edge_type != OpType::FLIGHT)
    {
      continue;
    }

    double value = 0.;

    for(const auto& variable : variables(edge))
    {
      value += SCIPgetSolVal(scip, solution, variable->get_var());
    }

    if(!SCIPisFeasLE(scip, value, 1.))
    {
      return true;
    }
  }

  return false;
}

SCIP_DECL_CONSTRANS(CapacityConstraintHandler::scip_trans)
{
  SCIP_CONSDATA* targetdata;

  targetdata = NULL;

  std::ostringstream namebuf;

  namebuf << "t_" << SCIPconsGetName(sourcecons);

  SCIP_CALL_EXC(SCIPcreateCons(scip,
                               targetcons,
                               namebuf.str().c_str(),
                               conshdlr,
                               targetdata,
                               SCIPconsIsInitial(sourcecons),
                               SCIPconsIsSeparated(sourcecons),
                               SCIPconsIsEnforced(sourcecons),
                               SCIPconsIsChecked(sourcecons),
                               SCIPconsIsPropagated(sourcecons),
                               SCIPconsIsLocal(sourcecons),
                               SCIPconsIsModifiable(sourcecons),
                               SCIPconsIsDynamic(sourcecons),
                               SCIPconsIsRemovable(sourcecons),
                               SCIPconsIsStickingAtNode(sourcecons)));

  return SCIP_OKAY;
}

SCIP_DECL_CONSENFOLP(CapacityConstraintHandler::scip_enfolp)
{
  *result = SCIP_FEASIBLE;

  if(is_violated())
  {
    *result = SCIP_INFEASIBLE;
  }

  return SCIP_OKAY;
}

SCIP_DECL_CONSENFOPS(CapacityConstraintHandler::scip_enfops)
{
  *result = SCIP_FEASIBLE;

  if(is_violated())
  {
    *result = SCIP_INFEASIBLE;
  }

  return SCIP_OKAY;
}

SCIP_DECL_CONSCHECK(CapacityConstraintHandler::scip_check)
{
  *result = SCIP_FEASIBLE;

  if(is_violated(sol))
  {
    *result = SCIP_INFEASIBLE;
  }

  return SCIP_OKAY;
}

SCIP_DECL_CONSLOCK(CapacityConstraintHandler::scip_lock)
{
  return SCIP_OKAY;
}

SCIP_DECL_CONSSEPALP(CapacityConstraintHandler::scip_sepalp)
{
  *result = SCIP_DIDNOTFIND;

  if(separate(conshdlr))
  {
    *result = SCIP_SEPARATED;
  }

  return SCIP_OKAY;
}

SCIP_DECL_CONSSEPASOL(CapacityConstraintHandler::scip_sepasol)
{
  *result = SCIP_DIDNOTFIND;

  if(separate(conshdlr, sol))
  {
    *result = SCIP_SEPARATED;
  }

  return SCIP_OKAY;
}

SCIP_DECL_CONSEXITSOL(CapacityConstraintHandler::scip_exitsol)
{
  for(const Edge& edge : graph.get_edges())
  {
    if(capacity_constraints(edge))
    {
      SCIP_CALL_EXC(SCIPreleaseRow(scip, &capacity_constraints(edge)));
      capacity_constraints(edge) = nullptr;
    }
  }

  return SCIP_OKAY;
}
