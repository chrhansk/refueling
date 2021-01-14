#include "lp_observer.hh"

#include "scip_utils.hh"

LPObserver::LPObserver(SCIP* scip)
  : scip::ObjEventhdlr(scip,
                       "observer",
                       "Observes the solving process"),
    scip(scip),
    num_lps(0),
    num_rows(0),
    num_cols(0),
    max_rows(0),
    max_cols(0),
    min_rows(inf),
    min_cols(inf),
    root_objval(-1),
    root_time(-1),
    root_solved(false)
{
}

SCIP_DECL_EVENTINITSOL(LPObserver::scip_initsol)
{
  SCIP_CALL_EXC(SCIPcatchEvent(scip, SCIP_EVENTTYPE_LPSOLVED, eventhdlr, NULL, NULL));

  SCIP_CALL_EXC(SCIPcreateClock(scip, &clock));

  SCIP_CALL_EXC(SCIPstartClock(scip, clock));

  return SCIP_OKAY;
}

SCIP_DECL_EVENTEXEC(LPObserver::scip_exec)
{
  idx rows = SCIPgetNLPRows(scip);
  idx cols = SCIPgetNLPCols(scip);

  ++num_lps;

  num_rows += rows;
  num_cols += cols;

  max_rows = std::max(max_rows, rows);
  max_cols = std::max(max_cols, cols);

  min_rows = std::min(min_rows, rows);
  min_cols = std::min(min_cols, cols);

  if(!root_solved)
  {
    root_solved = true;
    SCIPstopClock(scip, clock);

    root_objval = SCIPgetLPObjval(scip);

    root_time = SCIPgetClockTime(scip, clock);

    assert(root_objval != SCIP_INVALID);
  }


  return SCIP_OKAY;
}

SCIP_DECL_EVENTEXITSOL(LPObserver::scip_exitsol)
{
  SCIP_CALL_EXC(SCIPfreeClock(scip, &clock));

  SCIP_CALL(SCIPdropEvent(scip, SCIP_EVENTTYPE_LPSOLVED, eventhdlr, NULL, -1) );

  return SCIP_OKAY;
}

LPStats LPObserver::get_stats() const
{
  return LPStats{get_num_lps(),
      get_avg_rows(),
      get_avg_cols(),
      get_max_rows(),
      get_max_cols(),
      get_min_rows(),
      get_min_cols(),
      get_root_objval(),
      get_root_time(),
      get_root_solved()
      };
}
