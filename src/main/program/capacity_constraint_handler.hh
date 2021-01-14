#ifndef CAPACITY_CONSTRAINT_HANDLER_HH
#define CAPACITY_CONSTRAINT_HANDLER_HH

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "scheduling_program.hh"

#include "scip_utils.hh"

class CapacityConstraintHandler : public scip::ObjConshdlr
{
private:
  SchedulingProgram& program;
  SCIP* scip;

  const RequestGraph& request_graph;
  const Graph& graph;

  EdgeMap<SCIP_ROW*> capacity_constraints;

  bool separate(SCIP_CONSHDLR* conshdlr,
                SCIP_SOL* solution = nullptr);

  bool is_violated(SCIP_SOL* solution = nullptr);

  void create_capacity_constraint(SCIP_CONSHDLR* conshdlr,
                                  const Edge& edge);

public:
  CapacityConstraintHandler(SchedulingProgram& program);

  void added_path_variable(const PathVariable& variable);

  void add_dual_values(EdgeMap<double>& dual_values,
                       DualCostType cost_type) const;

  std::string get_name() const;

  virtual SCIP_DECL_CONSTRANS(scip_trans) override;

  virtual SCIP_DECL_CONSENFOLP(scip_enfolp) override;
  virtual SCIP_DECL_CONSENFOPS(scip_enfops) override;

  virtual SCIP_DECL_CONSCHECK(scip_check) override;

  virtual SCIP_DECL_CONSLOCK(scip_lock) override;

  virtual SCIP_DECL_CONSSEPALP(scip_sepalp) override;
  virtual SCIP_DECL_CONSSEPASOL(scip_sepasol) override;

  virtual SCIP_DECL_CONSEXITSOL(scip_exitsol) override;
};


#endif /* CAPACITY_CONSTRAINT_HANDLER_HH */
