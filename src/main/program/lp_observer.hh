#ifndef LP_OBSERVER_HH
#define LP_OBSERVER_HH

#include <objscip/objscip.h>

#include "util.hh"
#include "solution_stats.hh"

class LPObserver : public scip::ObjEventhdlr
{
private:
  SCIP* scip;

  SCIP_CLOCK* clock;

  idx num_lps;
  idx num_rows;
  idx num_cols;

  idx max_rows;
  idx max_cols;

  idx min_rows;
  idx min_cols;

  double root_objval;
  double root_time;
  bool root_solved;

public:
  LPObserver(SCIP* scip);

  virtual SCIP_DECL_EVENTINITSOL(scip_initsol) override;

  virtual SCIP_DECL_EVENTEXEC(scip_exec) override;

  virtual SCIP_DECL_EVENTEXITSOL(scip_exitsol) override;

  idx get_num_lps() const
  {
    return num_lps;
  }

  idx get_num_rows() const
  {
    return num_rows;
  }

  idx get_num_cols() const
  {
    return num_cols;
  }

  idx get_max_rows() const
  {
    return max_rows;
  }

  idx get_max_cols() const
  {
    return max_cols;
  }

  idx get_min_rows() const
  {
    return min_rows;
  }

  idx get_min_cols() const
  {
    return min_cols;
  }

  double get_avg_rows() const
  {
    return get_num_rows() / ((double) get_num_lps());
  }

  double get_avg_cols() const
  {
    return get_num_cols() / ((double) get_num_lps());
  }

  double get_root_objval() const
  {
    return root_objval;
  }

  double get_root_time() const
  {
    return root_time;
  }

  bool get_root_solved() const
  {
    return root_solved;
  }

  LPStats get_stats() const;

};


#endif /* LP_OBSERVER_HH */
