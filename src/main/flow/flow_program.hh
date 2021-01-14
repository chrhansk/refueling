#ifndef FLOW_PROGRAM_HH
#define FLOW_PROGRAM_HH

#include "program/program.hh"
#include "flow_graph.hh"

class FlowProgram : public Program
{
private:
  const FlowGraph& flow_graph;
  const Graph& graph;
  const Instance& instance;

  EdgeMap<SCIP_VAR*> flow_variables;
  VertexMap<SCIP_CONS*> flow_constraints;

  void create_flow_constraints();
  void create_flow_variables();

public:
  FlowProgram(const FlowGraph& flow_graph,
              const Settings& settings = Settings());

  void solve();

  ~FlowProgram();
};


#endif /* FLOW_PROGRAM_HH */
