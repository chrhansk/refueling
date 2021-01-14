#ifndef PATH_PRICER_HH
#define PATH_PRICER_HH

#include <memory>
#include <unordered_map>
#include <unordered_set>

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "program/scip_utils.hh"

#include "program/scheduling_program.hh"

#include "program/dual_values.hh"

#include "program/artificial_variable.hh"
#include "program/path_variable.hh"

#include "fuel_cost_function.hh"

class PathPricer : public scip::ObjPricer
{
public:
  class PricingResult
  {
  private:
    std::vector<Path> paths;
    double lower_bound;

  public:
    PricingResult(const std::vector<Path>& paths,
                  double lower_bound)
      : paths(paths),
        lower_bound(lower_bound)
    {}

    PricingResult(const std::vector<Path>& paths)
      : paths(paths),
        lower_bound(-inf)
    {}

    PricingResult(double lower_bound)
      : lower_bound(lower_bound)
    {}

    PricingResult()
      : lower_bound(-inf)
    {}

    const std::vector<Path>& get_paths() const
    {
      return paths;
    }

    double get_lower_bound() const
    {
      return lower_bound;
    }

  };

protected:
  struct SpecialPathVariables
  {
    std::vector<std::shared_ptr<PathVariable>> forbidden_paths, required_paths, entire_paths;
  };

  SpecialPathVariables get_special_path_variables() const;

  const RequestGraph& request_graph;
  const Graph& graph;
  const Instance& instance;

  SchedulingProgram& program;
  SCIP* scip;

  double adjust_bound(double bound) const;

  std::vector<ArtificialVariable> artificial_variables;

  std::unordered_map<Path, std::shared_ptr<PathVariable>> path_variables;

  EdgeMap<std::vector<std::shared_ptr<PathVariable>>> edge_variables;

  VertexMap<SCIP_CONS*> covering_constraints;

  SCIP_CONS* path_constraint;
  bool use_artificial_variables;
  bool initiated;

  const FuelCostFunction<double>& costs;

  void add_artificial_variable(const Request& request);

  EdgeSet get_forbidden_edges(const std::vector<std::shared_ptr<PathVariable>>& required_path_variables) const;

public:
  PathPricer(SchedulingProgram& program,
             const FuelCostFunction<double>& costs,
             bool use_artificial_variables = true);

  virtual SCIP_DECL_PRICERINIT(scip_init) override;

  virtual SCIP_DECL_PRICEREXIT(scip_exit) override;

  virtual SCIP_DECL_PRICERFARKAS(scip_farkas) override;

  virtual SCIP_DECL_PRICERREDCOST(scip_redcost) override;

  bool has_path(const Path& path) const;

  std::shared_ptr<PathVariable> add_path(const Path& path,
                                         bool priced = true);

  std::string get_name();

  VertexMap<SCIP_CONS*>& get_covering_constraints()
  {
    return covering_constraints;
  }

  const VertexFunc<SCIP_CONS*>& get_covering_constraints() const
  {
    return covering_constraints;
  }

  SCIP_CONS* get_path_constraint() const
  {
    return path_constraint;
  }

  const SchedulingProgram& get_program() const
  {
    return program;
  }

  const FuelCostFunction<double>& get_costs() const
  {
    return costs;
  }

  const std::vector<ArtificialVariable>& get_artificial_variables() const
  {
    return artificial_variables;
  }

  const std::unordered_map<Path, std::shared_ptr<PathVariable>>& get_path_variables() const
  {
    return path_variables;
  }

  virtual PricingResult perform_pricing(DualCostType cost_type) = 0;

  const EdgeFunc<std::vector<std::shared_ptr<PathVariable>>>& edge_path_variables() const;

  EdgeMap<double> get_combined_values(SCIP_SOL* solution = nullptr) const;

  std::shared_ptr<PathVariable> get_path_variable(const Path& path) const;

  bool add_solution(const std::vector<std::shared_ptr<PathVariable>>& solution,
                    SCIP_HEUR* heur = nullptr);

  const RequestGraph& get_request_graph() const
  {
    return request_graph;
  }
};




#endif /* PATH_PRICER_HH */
