#ifndef SIMPLE_PATH_PRICER_HH
#define SIMPLE_PATH_PRICER_HH

#include "path_pricer.hh"

#include "program/reduced_costs.hh"

#include "bounded_router/bounded_router.hh"
#include "bounded_router/forbidden_path_router.hh"

class SimplePathPricer : public PathPricer
{
private:
  const Parameters& parameters;
  BoundedRouter router;

  ForbiddenPathRouter forbidden_path_router;

  std::vector<Edge> request_edges;

  PathPricer::PricingResult compute_simple_bound(double upper_bound,
                                                 const BoundedRouter::Result& result) const;

public:
  SimplePathPricer(SchedulingProgram& program,
                     const FuelCostFunction<double>& costs,
                     bool use_artificial_variables = true);

  virtual PricingResult perform_pricing(DualCostType cost_type) override;
};

#endif /* SIMPLE_PATH_PRICER_HH */
