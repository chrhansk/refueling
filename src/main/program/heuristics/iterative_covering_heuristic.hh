#ifndef ITERATIVE_COVERING_HEURISTIC_HH
#define ITERATIVE_COVERING_HEURISTIC_HH

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "program/pricer/path_pricer.hh"

class IterativeCoveringHeuristic : public scip::ObjHeur
{
private:
  SchedulingProgram& program;
  PathPricer& pricer;
  const RequestGraph& request_graph;
  const Graph& graph;

  struct RequestPath
  {
    Path path;
    RequestPattern pattern;
  };

public:
  IterativeCoveringHeuristic(SchedulingProgram& program,
                             PathPricer& pricer);

  SCIP_DECL_HEUREXEC(scip_exec) override;
};


#endif /* ITERATIVE_COVERING_HEURISTIC_HH */
