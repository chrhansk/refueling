#ifndef FUNDAMENTAL_PATH_PRICER_HH
#define FUNDAMENTAL_PATH_PRICER_HH

#include <unordered_set>

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include <objscip/objscip.h>

#include "program/scip_utils.hh"

#include "fundamental_path/fundamental_path.hh"
#include "fundamental_path/fundamental_path_program.hh"

#include "fundamental_path/pricer/fundamental_path_dual_values.hh"

class FundamentalPathPricer : public scip::ObjPricer
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
  FundamentalPathProgram& program;
  SCIP* scip;
  const FundamentalPathGraph& fundamental_path_graph;
  const Graph& graph;
  const Instance& instance;

  struct SpecialPaths
  {
    std::vector<std::shared_ptr<FundamentalPath>> forbidden_paths, required_paths, entire_paths;
  };

  SpecialPaths get_special_paths() const;

  EdgeSet get_forbidden_edges(const std::vector<std::shared_ptr<FundamentalPath>>& required_paths) const;

  std::unordered_map<Path, std::shared_ptr<FundamentalPath>> paths;

  EdgeMap<std::vector<std::shared_ptr<FundamentalPath>>> edge_paths;

  EdgeMap<SCIP_CONS*> covering_constraints;

  const RequestMap<Edge>& request_edges;

  const FundamentalPath& add_path(const Path& path,
                                  const TimeSpan& timespan);

public:
  FundamentalPathPricer(FundamentalPathProgram& program);

  virtual SCIP_DECL_PRICERINIT(scip_init) override;

  virtual SCIP_DECL_PRICEREXIT(scip_exit) override;

  virtual SCIP_DECL_PRICERFARKAS(scip_farkas) override;

  virtual SCIP_DECL_PRICERREDCOST(scip_redcost) override;

  virtual PricingResult perform_pricing(DualCostType cost_type) = 0;

  SCIP_SOL* create_solution(const std::vector<Path>& paths,
                            SCIP_HEUR* heur = nullptr);

  std::string get_name() const;

  bool has_path(const Path& path) const;

  void add_dual_values(EdgeMap<double>& dual_values,
                       DualCostType cost_type) const;

  const std::unordered_map<Path, std::shared_ptr<FundamentalPath>>& get_paths() const
  {
    return paths;
  }

  const EdgeMap<std::vector<std::shared_ptr<FundamentalPath>>>& get_edge_paths() const
  {
    return edge_paths;
  }

  const EdgeMap<SCIP_CONS*>& get_covering_constraints() const
  {
    return covering_constraints;
  }
};


#endif /* FUNDAMENTAL_PATH_PRICER_HH */
