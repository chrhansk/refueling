#include "flow_program.hh"

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include "program/scip_utils.hh"

FlowProgram::FlowProgram(const FlowGraph& flow_graph,
                         const Settings& settings)
  : Program("flow", settings),
    flow_graph(flow_graph),
    graph(flow_graph),
    instance(flow_graph.get_instance()),
    flow_variables(graph, nullptr),
    flow_constraints(graph, nullptr)
{
  create_flow_variables();
  create_flow_constraints();
}

void FlowProgram::create_flow_constraints()
{
  VertexMap<double> demands(graph, 0);

  for(const Request& request : instance.get_requests())
  {
    demands(flow_graph.get_origin_vertices()(request)) = 1;
    demands(flow_graph.get_destination_vertices()(request)) = -1;
  }

  for(const Vertex&  vertex : graph.get_vertices())
  {
    std::ostringstream namebuf;
    SCIP_CONS* cons;

    namebuf << "demand_" << vertex;

    double demand = demands(vertex);

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &cons,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       demand,  // lhs
                                       demand,  // rhs
                                       TRUE,    // initial
                                       TRUE,    // separate
                                       TRUE,    // enforce
                                       TRUE,    // check
                                       TRUE,    // propagate
                                       FALSE,   // local
                                       FALSE,   // modifiable
                                       FALSE,   // dynamic
                                       FALSE,   // removable
                                       FALSE)); // sticking at node

    for(const Edge& incoming : graph.get_incoming(vertex))
    {
      SCIP_CALL_EXC(SCIPaddCoefLinear(scip,
                                      cons,
                                      flow_variables(incoming),
                                      1.0));
    }

    for(const Edge& outgoing : graph.get_outgoing(vertex))
    {
      SCIP_CALL_EXC(SCIPaddCoefLinear(scip,
                                      cons,
                                      flow_variables(outgoing),
                                      -1.0));
    }

    SCIP_CALL_EXC(SCIPaddCons(scip, cons));

    flow_constraints(vertex) = cons;
  }

}
void FlowProgram::create_flow_variables()
{
  for(const Edge& edge : graph.get_edges())
  {
    SCIP_VAR* var;

    std::ostringstream namebuf;

    namebuf << "x_" << edge.get_source() << "_" << edge.get_target();

    double cost = int(edge == flow_graph.get_cycle_edge());

    OpType edge_type = flow_graph.get_edge_types()(edge);

    double upper_bound =  1;

    if(edge_type == OpType::BASE_WAITING ||
       edge_type == OpType::BASE_REFUELING)
    {
      upper_bound = SCIPinfinity(scip);
    }

    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &var,
                                namebuf.str().c_str(),
                                0.0,
                                upper_bound,
                                cost,
                                SCIP_VARTYPE_CONTINUOUS,
                                TRUE,   // initial
                                FALSE,   // removable
                                NULL, NULL, NULL, NULL, NULL));

    SCIP_CALL_EXC(SCIPaddVar(scip, var));

    flow_variables(edge) = var;
  }

}


void FlowProgram::solve()
{
  SCIP_CALL_EXC(SCIPsolve(scip));

  write("flow.lp");
}

FlowProgram::~FlowProgram()
{
  for(const Vertex& vertex : graph.get_vertices())
  {
    SCIP_CONS* cons = flow_constraints(vertex);

    if(cons)
    {
      SCIP_CALL_ASSERT(SCIPreleaseCons(scip, &cons));
    }
  }

  flow_constraints.reset(nullptr);

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_VAR* var = flow_variables(edge);

    if(var)
    {
      SCIP_CALL_ASSERT(SCIPreleaseVar(scip, &var));
    }
  }

  flow_variables.reset(nullptr);
}
