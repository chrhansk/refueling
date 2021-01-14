#ifndef SCHEDULING_PROGRAM_HH
#define SCHEDULING_PROGRAM_HH


#include "program.hh"

#include "request_graph.hh"
#include "graph/edge_set.hh"

#include "dual_values.hh"
#include "scip_utils.hh"

#include "program/request_pattern.hh"

#include "solution.hh"

class CapacityConstraintHandler;

class FixedEdge;
class PathVariable;
class PathPricer;
class EdgePricer;

class SchedulingProgram : public Program
{
private:
  const RequestGraph& request_graph;
  const Graph& graph;
  const Instance& instance;
  int max_num_paths;

  EdgeMap<SCIP_VAR*> combined_variables;
  VertexMap<SCIP_CONS*> covering_constraints;
  SCIP_CONS* path_constraint;

  PathPricer* pricer;
  EdgePricer* edge_pricer;

  CapacityConstraintHandler* constraint_handler;

  void create_combined_variables();
  void create_covering_constraints();

  void print_solution(PathPricer* pricer, SCIP_SOL* sol) const;
  idx print_path(idx index, const Path& path) const;

  Path shortcut_path(const Path& path,
                     RequestPattern& pattern) const;

  void add_variable_to_solution(Solution& solution,
                                const PathVariable& path_variable) const;

public:
  SchedulingProgram(const RequestGraph& request_graph,
                    int max_num_paths = -1,
                    const Settings& settings = Settings());

  ~SchedulingProgram();

  bool solve(PathPricer* pricer,
             int time_limit = -1);

  Solution get_solution() const;

  const RequestGraph& get_request_graph() const
  {
    return request_graph;
  }

  const Graph& get_graph() const
  {
    return request_graph;
  }

  SCIP_CONS* get_path_constraint() const
  {
    return path_constraint;
  }

  PathPricer& get_pricer()
  {
    assert(pricer);
    return *pricer;
  }

  const PathPricer& get_pricer() const
  {
    assert(pricer);
    return *pricer;
  }

  const Instance& get_instance() const
  {
    return instance;
  }

  const VertexMap<SCIP_CONS*>& get_covering_constraints() const
  {
    return covering_constraints;
  }

  EdgeSet get_forbidden_edges() const;

  void added_path_variable(const PathVariable& variable);

  num get_max_num_paths() const
  {
    num num_requests = get_instance().get_requests().size();

    if(max_num_paths == -1)
    {
      return num_requests;
    }
    return std::min(max_num_paths, (num) get_instance().get_requests().size());
  }

  const std::unordered_map<Path, std::shared_ptr<PathVariable>>& get_path_variables() const;

  const EdgeFunc<std::vector<std::shared_ptr<PathVariable>>>& edge_path_variables() const;

  DualValues get_dual_values(DualCostType cost_type,
                             const std::vector<std::shared_ptr<PathVariable>>& entire_path_variables) const;

  EdgeMap<double> total_flow(SCIP_SOL* sol = nullptr) const;

  void add_solution(const Solution& solution, SCIP_HEUR* heur = nullptr);

  bool has_fixed_edges() const;
};


#endif /* SCHEDULING_PROGRAM_HH */
