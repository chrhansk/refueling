#ifndef FUNDAMENTAL_PATH_PROGRAM_HH
#define FUNDAMENTAL_PATH_PROGRAM_HH

#include <unordered_set>

#include "graph/edge_set.hh"

#include "program/program.hh"
#include "program/scip_utils.hh"

#include "fuel_cost_function.hh"

#include "fundamental_path.hh"
#include "fundamental_path_graph.hh"

#include "pricer/fundamental_path_dual_values.hh"

class FundamentalEdgePricer;
class FundamentalPathPricer;

class FundamentalPathProgram : public Program
{
public:

  class CliqueConstraint
  {
  private:
    SCIP_CONS* constraint;
    EdgeSet edges;

  public:
    CliqueConstraint(SCIP_CONS* constraint,
                     const EdgeSet& edges)
      : constraint(constraint),
        edges(edges)
    {}

    SCIP_CONS* get_constraint() const
    {
      return constraint;
    }

    void set_constraint(SCIP_CONS* cons)
    {
      constraint = cons;
    }

    const EdgeSet& get_edges() const
    {
      return edges;
    }

  };

private:
  const FundamentalPathGraph& fundamental_path_graph;
  const Graph& graph;
  const Instance& instance;

  const FuelCostFunction<double>& costs;

  FundamentalEdgePricer* edge_pricer;
  FundamentalPathPricer* fundamental_path_pricer;

  EdgeMap<SCIP_CONS*> covering_constraints;

  std::vector<CliqueConstraint> clique_constraints;

  SCIP_VAR* clique_variable;

  int max_num_paths;

  void create_covering_constraints();

  void create_clique_constraints();
  void create_clique_constraint(const EdgeSet& edges);

public:
  FundamentalPathProgram(const FundamentalPathGraph& fundamental_path_graph,
                         const FuelCostFunction<double>& costs,
                         int max_num_paths = -1,
                         const Settings& settings = Settings());

  ~FundamentalPathProgram();

  const FuelCostFunction<double>& get_costs() const
  {
    return costs;
  }

  const FundamentalPathGraph& get_fundamental_path_graph() const
  {
    return fundamental_path_graph;
  }

  const EdgeMap<SCIP_CONS*>& get_covering_constraints() const
  {
    return covering_constraints;
  }

  const Instance& get_instance() const
  {
    return instance;
  }

  num get_max_num_paths() const;

  EdgeSet get_forbidden_edges() const;

  const std::vector<CliqueConstraint>& get_clique_constraints() const
  {
    return clique_constraints;
  }

  SCIP_VAR* get_clique_variable() const
  {
    return clique_variable;
  }

  EdgeMap<double> total_flow(SCIP_SOL* sol = nullptr) const;

  const EdgeMap<std::vector<std::shared_ptr<FundamentalPath>>>& get_edge_paths() const;

  bool solve(int time_limit = -1);

  void add_solution(const Solution& solution,
                    SCIP_HEUR* heur = nullptr);

  Solution get_solution() const;

  void added_path(const FundamentalPath& path);

  FundamentalPathDualValues get_dual_values(DualCostType cost_type,
                                            const std::vector<std::shared_ptr<FundamentalPath>>& entire_paths) const;

  virtual void init_sol() override;
};


#endif /* FUNDAMENTAL_PATH_PROGRAM_HH */
