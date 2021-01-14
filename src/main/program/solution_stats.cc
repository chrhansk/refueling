#include "solution_stats.hh"

LPStats LPStats::empty()
{
  return LPStats{0,
      0, 0, 0, 0,
      0, 0, 0, 0};
}

SolutionStats::SolutionStats(SCIP* scip,
                             const LPStats& lp_stats)
  : lp_stats(lp_stats)
{
  primal_bound = SCIPgetPrimalbound(scip);
  dual_bound = SCIPgetDualbound(scip);
  gap = SCIPgetGap(scip);

  max_depth = SCIPgetMaxDepth(scip);
  num_iterations = SCIPgetNLPIterations(scip);
  num_nodes = SCIPgetNNodes(scip);

  num_priced_variables = SCIPgetNPricevars(scip);
  num_variables = SCIPgetNVars(scip);
  num_constraints = SCIPgetNConss(scip);
  num_cuts = SCIPgetNPoolCuts(scip);
  separation_time = SCIPcutpoolGetTime(SCIPgetGlobalCutpool(scip));
}

SolutionStats SolutionStats::empty()
{
  return SolutionStats();
}

SolutionStats::SolutionStats()
  : lp_stats(LPStats::empty()),
    primal_bound(0),
    dual_bound(0),
    gap(0),
    max_depth(0),
    num_iterations(0),
    num_nodes(0),
    num_priced_variables(0),
    num_variables(0),
    num_constraints(0),
    num_cuts(0),
    separation_time(0)
{
}
