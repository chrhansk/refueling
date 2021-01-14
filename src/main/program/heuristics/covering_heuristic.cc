#include "covering_heuristic.hh"

#include "log.hh"

#include "greedy_covering.hh"

#define NAME "covering_heuristic"

CoveringHeuristic::CoveringHeuristic(SchedulingProgram& program,
                                     PathPricer& pricer)
  : scip::ObjHeur(program.getSCIP(),                 // SCIP
                  NAME,                              // name
                  "Computes greedy coverings",       // description
                  'g'   ,                            // display character
                  10000,                             // priority
                  1,                                 // frequency
                  0,                                 // frequency offset
                  -1,                                // max depth
                  SCIP_HEURTIMING_DURINGPRICINGLOOP, // timing
                  FALSE),                            // subscip
  program(program),
  pricer(pricer),
  request_graph(program.get_request_graph()),
  current_iteration(0)
{

}

SCIP_DECL_HEUREXEC(CoveringHeuristic::scip_exec)
{
  if((++current_iteration % 50) != 0)
  {
    return SCIP_OKAY;
  }

  GreedyCovering greedy_covering(request_graph);

  for(const auto& pair : pricer.get_path_variables())
  {
    const PathVariable& variable = *(pair.second);

    if(cmp::pos(SCIPgetVarSol(scip, variable.get_var())))
    {
      greedy_covering.add_path(variable.get_path(),
                               variable.get_cost(),
                               variable.get_pattern());
    }
  }

  auto covering = greedy_covering.compute(program.get_max_num_paths());

  *result = SCIP_DIDNOTFIND;

  if(!covering)
  {
    return SCIP_OKAY;
  }

  if(cmp::ge(covering.get_cost(), SCIPgetCutoffbound(scip)))
  {
    return SCIP_OKAY;
  }

  std::vector<std::shared_ptr<PathVariable>> solution_variables;

  for(const Path& path : covering.get_paths())
  {
    solution_variables.push_back(pricer.get_path_variable(path));
  }

  if(pricer.add_solution(solution_variables, heur))
  {
    *result = SCIP_FOUNDSOL;
  }

  return SCIP_OKAY;
}
