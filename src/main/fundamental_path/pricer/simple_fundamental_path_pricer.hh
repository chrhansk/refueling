#ifndef SIMPLE_FUNDAMENTAL_PATH_PRICER_HH
#define SIMPLE_FUNDAMENTAL_PATH_PRICER_HH

#include "fundamental_path_pricer.hh"

#include "bounded_router/bounded_router.hh"
#include "bounded_router/forbidden_path_router.hh"

class SimpleFundamentalPathPricer : public FundamentalPathPricer
{
private:
  BoundedRouter router;

  ForbiddenPathRouter forbidden_path_router;

  bool initiated;

public:
  SimpleFundamentalPathPricer(FundamentalPathProgram& program);

  virtual PricingResult perform_pricing(DualCostType cost_type) override;
};


#endif /* SIMPLE_FUNDAMENTAL_PATH_PRICER_HH */
