#include "scip_utils.hh"

#include <scip/scipdefplugins.h>

double cons_dual_value(SCIP* scip,
                       SCIP_CONS* cons,
                       DualCostType cost_type)
{
  if(!cons)
  {
    return 0;
  }

  if(cost_type == DualCostType::FARKAS)
  {
    return SCIPgetDualfarkasLinear(scip, cons);
  }
  else
  {
    assert(cost_type == DualCostType::SIMPLE);

    return SCIPgetDualsolLinear(scip, cons);
  }
}

double row_dual_value(SCIP_ROW* row,
                      DualCostType cost_type)
{
  if(!row)
  {
    return 0;
  }

  if(cost_type == DualCostType::FARKAS)
  {
    return SCIProwGetDualfarkas(row);
  }
  else
  {
    assert(cost_type == DualCostType::SIMPLE);

    return SCIProwGetDualsol(row);
  }


}
