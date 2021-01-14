#ifndef SOLUTION_STATS_HH
#define SOLUTION_STATS_HH

#include <scip/scip.h>

#include "util.hh"

struct LPStats
{
  idx num_lps;
  double avg_rows;
  double avg_cols;

  idx max_rows;
  idx max_cols;

  idx min_rows;
  idx min_cols;

  double root_obj_val;
  double root_time;
  bool root_lp_solved;

  static LPStats empty();
};

struct SolutionStats
{
private:
  SolutionStats();
public:
  SolutionStats(SCIP* scip, const LPStats& lp_stats);
  LPStats lp_stats;

  double primal_bound;
  double dual_bound;
  double gap;

  idx max_depth;
  idx num_iterations;
  idx num_nodes;

  idx num_priced_variables;
  idx num_variables;
  idx num_constraints;

  idx num_cuts;
  double separation_time;

  static SolutionStats empty();

};

#endif /* SOLUTION_STATS_HH */
