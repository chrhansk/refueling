#ifndef EDGE_PRICER_HH
#define EDGE_PRICER_HH

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "program/scip_utils.hh"

#include "program/scheduling_program.hh"

class EdgePricer : public scip::ObjPricer
{
protected:
  SchedulingProgram& program;
  SCIP* scip;
  const RequestGraph& request_graph;
  const Graph& graph;
  const Instance& instance;

  EdgeMap<SCIP_CONS*> linking_constraints;
  EdgeMap<SCIP_VAR*> compound_variables;

  struct EdgeVariable
  {
    Edge edge;
    SCIP_VAR* variable;
  };

  std::vector<EdgeVariable> edge_variables;

  void create_branching_candidate(const Edge& edge);

public:
  EdgePricer(SchedulingProgram& program);

  virtual SCIP_DECL_PRICERREDCOST(scip_redcost) override;

  virtual SCIP_DECL_PRICEREXIT(scip_init) override;

  virtual SCIP_DECL_PRICEREXIT(scip_exit) override;

  EdgeSet get_forbidden_edges() const;

  void added_path_variable(const PathVariable& variable);

  void add_dual_values(EdgeMap<double>& dual_values,
                       DualCostType cost_type) const;

  void set_compound_solution_values(const std::vector<Path>& paths,
                                    SCIP_SOL* sol);

  const EdgeMap<SCIP_CONS*>& get_linking_constraints() const
  {
    return linking_constraints;
  }
    const EdgeMap<SCIP_VAR*>& get_compound_variables() const
  {
    return compound_variables;
  }

  std::string get_name();

  bool has_fixed_edges() const;

};


#endif /* EDGE_PRICER_HH */
