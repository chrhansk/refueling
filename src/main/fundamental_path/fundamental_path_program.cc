#include "fundamental_path_program.hh"

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include "pricer/fundamental_edge_pricer.hh"
#include "pricer/simple_fundamental_path_pricer.hh"

#include "log.hh"

namespace
{
  enum class EventType
  {
    Entering,
    Leaving
  };

  struct Event
  {
    EventType type;
    DateTime time;
    Edge edge;

    bool operator<(const Event& other)
    {
      if(time != other.time)
      {
        return time < other.time;
      }

      if(type != other.type)
      {
        return type == EventType::Leaving && other.type == EventType::Entering;
      }

      return edge.get_index() < other.edge.get_index();
    }

  };
}

FundamentalPathProgram::FundamentalPathProgram(const FundamentalPathGraph& fundamental_path_graph,
                                               const FuelCostFunction<double>& costs,
                                               int max_num_paths,
                                               const Settings& settings)
  : Program("pattern", settings),
    fundamental_path_graph(fundamental_path_graph),
    graph(fundamental_path_graph),
    instance(fundamental_path_graph.get_instance()),
    costs(costs),
    covering_constraints(fundamental_path_graph, nullptr),
    max_num_paths(max_num_paths)
{
  double path_cost = (max_num_paths == - 1) ? 1. : 0.;

  SCIP_CALL_EXC(SCIPcreateVar(scip,
                              &clique_variable,
                              "b",
                              0.,                       // lower
                              get_max_num_paths(),      // upper
                              path_cost,                // cost
                              SCIP_VARTYPE_IMPLINT,     // vartype
                              TRUE,                     // initial
                              TRUE,                     // removable
                              NULL, NULL, NULL, NULL, NULL));

  SCIP_CALL_EXC(SCIPaddVar(scip, clique_variable));

  create_covering_constraints();

  if(max_num_paths < (int) get_instance().get_requests().size() || path_cost != 0.)
  {
    create_clique_constraints();
  }


  {
    fundamental_path_pricer = new SimpleFundamentalPathPricer(*this);

    SCIP_CALL_EXC(SCIPincludeObjPricer(scip, fundamental_path_pricer, true));

    SCIP_CALL_EXC(SCIPactivatePricer(scip, SCIPfindPricer(scip, fundamental_path_pricer->get_name().c_str())));
  }

  {
    edge_pricer = new FundamentalEdgePricer(*this);

    SCIP_CALL_EXC(SCIPincludeObjPricer(scip, edge_pricer, true));

    SCIP_CALL_EXC(SCIPactivatePricer(scip, SCIPfindPricer(scip, edge_pricer->get_name().c_str())));
  }

}

void FundamentalPathProgram::init_sol()
{
  {
    SCIP_VAR* transformed_variable;

    SCIP_CALL_EXC(SCIPgetTransformedVar(scip, clique_variable, &transformed_variable));

    SCIP_CALL_EXC(SCIPcaptureVar(scip, transformed_variable));

    SCIP_CALL_EXC(SCIPreleaseVar(scip, &clique_variable));

    clique_variable = transformed_variable;
  }

  for(CliqueConstraint& clique_constraint : clique_constraints)
  {
    SCIP_CONS* constraint = clique_constraint.get_constraint();

    if(constraint)
    {
      SCIP_CONS* transformed_constraint;

      SCIP_CALL_EXC(SCIPgetTransformedCons(scip, constraint, &transformed_constraint));

      SCIP_CALL_EXC(SCIPcaptureCons(scip, transformed_constraint));

      SCIP_CALL_EXC(SCIPreleaseCons(scip, &constraint));

      clique_constraint.set_constraint(transformed_constraint);
    }
  }
}

FundamentalPathProgram::~FundamentalPathProgram()
{
  SCIP_CALL_ASSERT(SCIPreleaseVar(scip, &clique_variable));

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_CONS* covering_constraint = covering_constraints(edge);

    if(covering_constraint)
    {
      SCIP_CALL_ASSERT(SCIPreleaseCons(scip, &covering_constraint));
      covering_constraints(edge) = nullptr;
    }

  }

  for(CliqueConstraint& clique_constraint : clique_constraints)
  {
    SCIP_CONS* cons = clique_constraint.get_constraint();

    if(cons)
    {
      SCIP_CALL_ASSERT(SCIPreleaseCons(scip, &cons));

      clique_constraint.set_constraint(nullptr);
    }
  }
}

num FundamentalPathProgram::get_max_num_paths() const
{
  num num_requests = get_instance().get_requests().size();

  if(max_num_paths == -1)
  {
    return num_requests;
  }
  return std::min(max_num_paths, (num) get_instance().get_requests().size());
}

EdgeSet FundamentalPathProgram::get_forbidden_edges() const
{
  return edge_pricer->get_forbidden_edges();
}

void FundamentalPathProgram::create_clique_constraints()
{
  std::vector<Event> events;

  const VertexMap<DateTime>& times = fundamental_path_graph.get_times();

  idx num_constraints = 0;

  Log(info) << "Creating capacity constraints";

  for(const Edge& edge : graph.get_edges())
  {
    if(edge.get_source() == fundamental_path_graph.get_origin() ||
       edge.get_target() == fundamental_path_graph.get_destination())
    {
      continue;
    }

    assert(times(edge.get_source()) < times(edge.get_target()));

    events.push_back(Event{EventType::Entering, times(edge.get_source()), edge});
    events.push_back(Event{EventType::Leaving, times(edge.get_target()), edge});
  }

  std::sort(std::begin(events), std::end(events));

  auto it = std::begin(events);
  auto end = std::end(events);

  EdgeSet edges(graph);

  for(; it != end;)
  {
    if(it->type == EventType::Leaving)
    {
      while(it != end && it->type == EventType::Leaving)
      {
        edges.remove(it->edge);

        ++it;
      }
    }
    else if(it->type == EventType::Entering)
    {
      while(it != end && it->type == EventType::Entering)
      {
        edges.insert(it->edge);

        ++it;
      }

      ++num_constraints;

      create_clique_constraint(edges);
    }
  }

  Log(info) << "Created " << num_constraints << " capacity constraints";

}

void FundamentalPathProgram::create_clique_constraint(const EdgeSet& edges)
{
  SCIP_CONS* clique_constraint;

  std::ostringstream namebuf;

  namebuf << "clique_" << clique_constraints.size();

  SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                     &clique_constraint,
                                     namebuf.str().c_str(),
                                     0,
                                     NULL,
                                     NULL,
                                     -SCIPinfinity(scip), // lhs
                                     0,                   // rhs
                                     TRUE,                // initial
                                     TRUE,                // separate
                                     TRUE,                // enforce
                                     TRUE,                // check
                                     TRUE,                // propagate
                                     FALSE,               // local
                                     TRUE,                // modifiable
                                     FALSE,               // dynamic
                                     FALSE,               // removable
                                     FALSE));             // sticking at node

  SCIP_CALL_EXC(SCIPaddCoefLinear(scip, clique_constraint, clique_variable, -1.));

  SCIP_CALL_EXC(SCIPaddCons(scip, clique_constraint));

  clique_constraints.push_back(CliqueConstraint(clique_constraint, edges));
}


void FundamentalPathProgram::create_covering_constraints()
{
  const RequestMap<Edge>& request_edges = fundamental_path_graph.get_request_edges();

  for(const Request& request : instance.get_requests())
  {
    std::ostringstream namebuf;
    SCIP_CONS* cons;

    namebuf << "covering_" << request.get_index();

    Edge edge = request_edges(request);

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &cons,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       1.,                 // lhs
                                       1.,                 // rhs
                                       TRUE,               // initial
                                       TRUE,               // separate
                                       TRUE,               // enforce
                                       TRUE,               // check
                                       TRUE,               // propagate
                                       FALSE,              // local
                                       TRUE,               // modifiable
                                       FALSE,              // dynamic
                                       FALSE,              // removable
                                       FALSE));            // sticking at node

    SCIP_CALL_EXC(SCIPaddCons(scip, cons));

    covering_constraints(edge) = cons;
  }
}

EdgeMap<double> FundamentalPathProgram::total_flow(SCIP_SOL* sol) const
{
  EdgeMap<double> flow(fundamental_path_graph, 0.);

  for(const auto& pair : fundamental_path_pricer->get_paths())
  {
    const auto& path = pair.second;

    const double value = SCIPgetSolVal(scip, sol, path->get_trip_variable());

    if(cmp::zero(value))
    {
      continue;
    }

    for(const Edge& edge : path->get_path().get_edges())
    {
      flow(edge) += value;
    }
  }

  return flow;
}

const EdgeMap<std::vector<std::shared_ptr<FundamentalPath>>>& FundamentalPathProgram::get_edge_paths() const
{
  return fundamental_path_pricer->get_edge_paths();
}

void FundamentalPathProgram::add_solution(const Solution& solution,
                                          SCIP_HEUR* heur)
{
  std::vector<Path> paths;

  for(const auto& solution_path : solution.get_paths())
  {
    std::vector<Path> current_paths = fundamental_path_graph.paths_for(solution_path);

    for(const Path& path : current_paths)
    {
      paths.push_back(path);
    }
  }

  if(debugging_enabled())
  {
    RequestPattern pattern(instance);

    for(const Path& path : paths)
    {
      for(const Edge& edge : path.get_edges())
      {
        if(fundamental_path_graph.get_edge_types()(edge) == OpType::REFUELING)
        {
          const auto& request = fundamental_path_graph.get_edge_requests()(edge);

          pattern.insert(request);
        }
      }
    }
    assert(pattern == RequestPattern::all(instance));
  }

  SCIP_SOL* sol = fundamental_path_pricer->create_solution(paths);

  {
    SCIP_VAR* clique_variable = get_clique_variable();

    double value = get_max_num_paths();

    value = std::max(value, SCIPvarGetLbLocal(clique_variable));
    value = std::min(value, SCIPvarGetUbLocal(clique_variable));

    SCIP_CALL_EXC(SCIPsetSolVal(scip, sol, clique_variable, value));
  }


  edge_pricer->set_compound_solution_values(paths, sol);

  SCIP_Bool stored;

  SCIP_CALL_EXC(SCIPaddSolFree(scip, &sol, &stored));

  Log(debug) << "Solution was stored?" << std::boolalpha << stored;
}

bool FundamentalPathProgram::solve(int time_limit)
{
  if(time_limit != -1)
  {
    SCIPsetRealParam(scip, "limits/time", time_limit);
  }

  if(get_costs().integral())
  {
    SCIP_CALL_EXC(SCIPsetObjIntegral(scip));
  }

  SCIPsetIntParam(scip, "presolving/maxrounds", 0);

  Log(info) << "Solving a problem with an upper bound of " << get_max_num_paths() << " paths";


  /*
    if(pricer->get_costs().integral())
    {
    SCIP_CALL_EXC(SCIPsetObjIntegral(scip));
    }
  */

  SCIP_CALL_EXC(SCIPsolve(scip));

  SCIP_SOL* solution = SCIPgetBestSol(scip);

  return !!solution;
}

Solution FundamentalPathProgram::get_solution() const
{
  Solution solution(SCIPgetGap(scip));

  SCIP_SOL* sol = SCIPgetBestSol(scip);

  assert(sol);

  std::vector<std::shared_ptr<FundamentalPath>> paths;

  for(const auto& pair : fundamental_path_pricer->get_paths())
  {
    const auto& path = pair.second;

    SCIP_VAR* trip_variable = path->get_trip_variable();

    double value = SCIPgetSolVal(scip, sol, trip_variable);

    if(value < 0.5)
    {
      continue;
    }

    paths.push_back(path);
  }

  std::sort(std::begin(paths),
            std::end(paths),
            [&](const std::shared_ptr<FundamentalPath>& first, const std::shared_ptr<FundamentalPath>& second) -> bool
            {
              return first->get_timespan().get_begin() < second->get_timespan().get_begin();
            });

  RequestPattern request_pattern(instance);

  for(const auto& fundamental_path : paths)
  {
    bool found = false;

    const Path& path = fundamental_path->get_path();

    auto current_solution_path = fundamental_path_graph.solution_path_for(path);

    const TimeSpan& current_timespan = current_solution_path.get_timespan();

    for(auto& solution_path : solution.get_paths())
    {
      Duration gap = current_timespan.get_begin() - solution_path.get_timespan().get_end();

      if(gap >= Time::seconds(0))
      {
        if(gap > Time::seconds(0))
        {
          TimeSpan waiting_timespan(solution_path.get_timespan().get_end(),
                                    current_timespan.get_begin());

          solution_path.append(Solution::Step::base_waiting(waiting_timespan,
                                                            instance.get_origin()));
        }

        solution_path.append(current_solution_path);

        found = true;

        break;
      }
    }

    if(!found)
    {
      solution.get_paths().push_back(current_solution_path);
    }
  }

  assert(solution.is_valid(instance, fundamental_path_graph.get_parameters()));

  return solution;
}

void FundamentalPathProgram::added_path(const FundamentalPath& pattern)
{
  edge_pricer->added_path(pattern);
}

FundamentalPathDualValues FundamentalPathProgram::get_dual_values(DualCostType cost_type,
                                                                  const std::vector<std::shared_ptr<FundamentalPath>>& entire_paths) const
{
  FundamentalPathDualValues dual_values(fundamental_path_graph);

  fundamental_path_pricer->add_dual_values(dual_values.get_edge_values(),
                                  cost_type);

  const auto& compound_variables = edge_pricer->get_compound_variables();

  const auto& covering_constraints = fundamental_path_pricer->get_covering_constraints();

  auto& edge_values = dual_values.get_edge_values();

  if(get_costs().nonnegative() && cost_type == DualCostType::SIMPLE)
  {
    for(const auto& entire_path : entire_paths)
    {
      const Path& path = entire_path->get_path();

      SCIP_VAR* var = entire_path->get_trip_variable();

      double reduced_cost = SCIPgetVarRedcost(scip, var);

      if(!SCIPisFeasNegative(scip, reduced_cost))
      {
        continue;
      }

      bool found_compound_variable = false;

      for(const Edge& edge : path.get_edges())
      {
        if(compound_variables(edge))
        {
          edge_values(edge) += reduced_cost;
          found_compound_variable = true;
          break;
        }
      }

      if(found_compound_variable)
      {
        continue;
      }

      bool found_covering_constraint = false;

      for(const Edge& edge : path.get_edges())
      {
        if(covering_constraints(edge))
        {
          edge_values(edge) += reduced_cost;
          found_covering_constraint = true;
          break;
        }
      }

      assert(found_covering_constraint);
    }
  }

  edge_pricer->add_dual_values(dual_values.get_edge_values(),
                               cost_type);

  return dual_values;
}
