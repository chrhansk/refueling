#ifndef COVERING_HEURISTIC_HH
#define COVERING_HEURISTIC_HH

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "program/pricer/path_pricer.hh"

class CoveringHeuristic : public scip::ObjHeur
{
private:
  SchedulingProgram& program;
  PathPricer& pricer;
  const RequestGraph& request_graph;
  idx current_iteration;

public:
  CoveringHeuristic(SchedulingProgram& program,
                    PathPricer& pricer);

  SCIP_DECL_HEUREXEC(scip_exec) override;
};


#endif /* COVERING_HEURISTIC_HH */
