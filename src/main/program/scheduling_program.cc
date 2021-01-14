#include "scheduling_program.hh"

#include <sstream>

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include "capacity_constraint_handler.hh"

#include "pricer/path_pricer.hh"
#include "pricer/edge_pricer.hh"

#include "log.hh"

#include "heuristics/covering_heuristic.hh"
#include "heuristics/iterative_covering_heuristic.hh"

#include "fuel_cost_function.hh"

SchedulingProgram::SchedulingProgram(const RequestGraph& request_graph,
                                     int max_num_paths,
                                     const Settings& settings)
  : Program("scheduling", settings),
    request_graph(request_graph),
    graph(request_graph),
    instance(request_graph.get_instance()),
    max_num_paths(max_num_paths),
    combined_variables(request_graph, nullptr),
    covering_constraints(request_graph, nullptr),
    path_constraint(nullptr),
    pricer(nullptr),
    edge_pricer(nullptr),
    constraint_handler(nullptr)
{
  /*
  create_combined_variables();
  */

  create_covering_constraints();

  if(max_num_paths != -1)
  {
    std::ostringstream namebuf;

    namebuf << "path_number";

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &path_constraint,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       -SCIPinfinity(scip),  // lhs
                                       max_num_paths,        // rhs
                                       TRUE,                 // initial
                                       TRUE,                 // separate
                                       TRUE,                 // enforce
                                       TRUE,                 // check
                                       TRUE,                 // propagate
                                       FALSE,                // local
                                       TRUE,                 // modifiable
                                       FALSE,                // dynamic
                                       FALSE,                // removable
                                       FALSE));              // sticking at node

    SCIP_CALL_EXC(SCIPaddCons(scip, path_constraint));
  }
}

SchedulingProgram::~SchedulingProgram()
{
  if(path_constraint)
  {
    SCIP_CALL_ASSERT(SCIPreleaseCons(scip, &path_constraint));
  }

  for(const Vertex& vertex : graph.get_vertices())
  {
    SCIP_CONS* cons = covering_constraints(vertex);

    if(cons)
    {
      SCIP_CALL_ASSERT(SCIPreleaseCons(scip, &cons));
    }
  }

  covering_constraints.reset(nullptr);

  for(const Edge& edge : graph.get_edges())
  {
    SCIP_VAR* var = combined_variables(edge);

    if(var)
    {
      SCIP_CALL_ASSERT(SCIPreleaseVar(scip, &var));
    }
  }

}

bool SchedulingProgram::solve(PathPricer* p, int time_limit)
{
  if(time_limit != -1)
  {
    SCIPsetRealParam(scip, "limits/time", time_limit);
  }

  assert(!pricer);
  pricer = p;

  SCIP_CALL_EXC(SCIPincludeObjPricer(scip, p, TRUE));

  SCIP_CALL_EXC(SCIPactivatePricer(scip, SCIPfindPricer(scip, p->get_name().c_str())));

  {
    edge_pricer = new EdgePricer(*this);

    SCIP_CALL_EXC(SCIPincludeObjPricer(scip, edge_pricer, TRUE));

    SCIP_CALL_EXC(SCIPactivatePricer(scip, SCIPfindPricer(scip, edge_pricer->get_name().c_str())));
  }

  pricer = p;

  {
    CoveringHeuristic* heur = new CoveringHeuristic(*this, *pricer);

    SCIP_CALL_EXC(SCIPincludeObjHeur(scip, heur, TRUE));
  }

  {
    IterativeCoveringHeuristic* heur = new IterativeCoveringHeuristic(*this, *pricer);

    SCIP_CALL_EXC(SCIPincludeObjHeur(scip, heur, TRUE));
  }

  /*
  {
    constraint_handler = new CapacityConstraintHandler(*this);

    SCIP_CALL_EXC(SCIPincludeObjConshdlr(scip, constraint_handler, TRUE));

    std::string handler_name = constraint_handler->get_name();

    SCIP_CONSHDLR* conshdlr = SCIPfindConshdlr(scip, handler_name.c_str());

    assert(conshdlr);

    SCIP_CONS* cons;

    SCIP_CALL_EXC(SCIPcreateCons(scip,
                                 &cons,
                                 handler_name.c_str(),
                                 conshdlr,
                                 NULL,
                                 FALSE, TRUE, TRUE, TRUE,
                                 TRUE, FALSE, FALSE, FALSE,
                                 TRUE, FALSE));

    SCIP_CALL_EXC(SCIPaddCons(scip, cons));
    SCIP_CALL_EXC(SCIPreleaseCons(scip, &cons));
  }
  */

  if(pricer->get_costs().integral())
  {
    SCIP_CALL_EXC(SCIPsetObjIntegral(scip));
  }

  SCIP_CALL_EXC(SCIPsolve(scip));

  SCIP_SOL* solution = SCIPgetBestSol(scip);

  if(!solution)
  {
    Log(info) << "Could not find a feasible solution";

    return false;
  }

  return true;
}

Solution SchedulingProgram::get_solution() const
{
  Solution solution(SCIPgetGap(scip));

  const auto& pricer = get_pricer();
  const auto& costs = pricer.get_costs();

  auto fuel_difference = request_graph.fuel_difference();
  const auto& parameters = request_graph.get_parameters();

  SCIP_SOL* sol = SCIPgetBestSol(scip);

  assert(sol);

  RequestPattern pattern(instance);

  for(const auto& pair : pricer.get_path_variables())
  {
    const auto& path_variable = pair.second;

    double value = SCIPgetSolVal(scip, sol, path_variable->get_var());

    if(value < 0.5)
    {
      continue;
    }

    Path path = path_variable->get_path();

    Path shortcut = shortcut_path(path, pattern);

    if(debugging_enabled())
    {
      const double path_cost = costs.cost(path, fuel_difference);
      const double shortcut_cost = costs.cost(path, fuel_difference);

      assert(cmp::le(shortcut_cost, path_cost));

      assert(fuel_difference.valid_path(shortcut,
                                        parameters.get_refueling_amount()));
    }

    solution.get_paths().push_back(request_graph.solution_path_for(shortcut));
  }

  assert(solution.is_valid(instance, request_graph.get_parameters()));

  return solution;
}

Path SchedulingProgram::shortcut_path(const Path& path,
                                      RequestPattern& pattern) const
{
  Path shortcut;

  auto prev = path.get_edges().begin();

  auto curr = prev;
  ++curr;

  auto next = curr;
  ++next;

  auto end = path.get_edges().end();

  auto edge_types = request_graph.get_edge_types();
  auto edge_requests = request_graph.get_edge_requests();

  shortcut.append(*prev);

  auto advance = [&]()
    {
      ++prev;
      ++curr;
      ++next;
    };

  auto get_edge= [&](const Vertex& vertex,
                     Direction direction,
                     OpType type) -> Edge
    {
      for(const Edge & edge : graph.get_edges(vertex, direction))
      {
        if(type == edge_types(edge))
        {
          return edge;
        }
      }

      assert(false);
    };

  while(next != end)
  {
    const Edge& curr_edge = *curr;
    const Edge& prev_edge = *prev;
    const Edge& next_edge = *next;

    bool needs_shortcut = false;

    if(edge_types(curr_edge) == OpType::REFUELING)
    {
      auto request = edge_requests(curr_edge);

      if(pattern.contains(request))
      {
        needs_shortcut = true;
      }
      else
      {
        pattern.insert(request);
      }
    }


    if(needs_shortcut)
    {
      shortcut.pop_back();

      const auto prev_type = edge_types(prev_edge);
      const auto next_type = edge_types(next_edge);

      const auto& target = next_edge.get_target();
      const auto& source = prev_edge.get_source();

      if(prev_type == OpType::FLIGHT &&
         next_type == OpType::FLIGHT)
      {
        bool found = false;

        for(const Edge& edge : graph.get_outgoing(source))
        {
          if(edge_types(edge) == OpType::FLIGHT &&
             edge.get_target() == target)
          {
            shortcut.append(edge);
            found = true;
            break;
          }
        }
        assert(found);
      }

      if(prev_type == OpType::FLIGHT &&
         next_type == OpType::DESCENT)
      {
        shortcut.append(get_edge(source,
                                 Direction::OUTGOING,
                                 OpType::DESCENT));

        while(shortcut.get_target() != target)
        {
          shortcut.append(get_edge(shortcut.get_target(),
                                   Direction::OUTGOING,
                                   OpType::BASE_WAITING));
        }

      }

      if(prev_type == OpType::CLIMB &&
         next_type == OpType::FLIGHT)
      {
        auto takeoff_edge = get_edge(target,
                                     Direction::INCOMING,
                                     OpType::CLIMB);

        while(shortcut.get_target() != takeoff_edge.get_source())
        {
          shortcut.append(get_edge(shortcut.get_target(),
                                   Direction::OUTGOING,
                                   OpType::BASE_WAITING));
        }

        shortcut.append(takeoff_edge);
      }

      if(prev_type == OpType::CLIMB &&
         next_type == OpType::DESCENT)
      {
        while(shortcut.get_target() != target)
        {
          shortcut.append(get_edge(shortcut.get_target(),
                                   Direction::OUTGOING,
                                   OpType::BASE_WAITING));
        }
      }

      assert(shortcut.get_target() == target);

      advance();

    }
    else
    {
      shortcut.append(curr_edge);
    }

    advance();
  }

  shortcut.append(*curr);

  assert(shortcut.connects(path.get_source(),
                           path.get_target()));

  return shortcut;
}

EdgeSet SchedulingProgram::get_forbidden_edges() const
{
  if(edge_pricer)
  {
    return edge_pricer->get_forbidden_edges();
  }

  return EdgeSet(graph);
}

void SchedulingProgram::added_path_variable(const PathVariable& variable)
{
  //assert(branching);

  //get_branching().added_path_variable(variable);

  if(edge_pricer)
  {
    edge_pricer->added_path_variable(variable);
  }

  if(constraint_handler)
  {
    constraint_handler->added_path_variable(variable);
  }

}

void SchedulingProgram::print_solution(PathPricer* pricer, SCIP_SOL* sol) const
{
  assert(sol);

  idx index = 0;

  const Parameters& parameters = request_graph.get_parameters();

  std::cout << "Efficiency: " << parameters.get_efficiency() << std::endl;

  std::cout << "Empty weight: " << parameters.get_empty_weight() << std::endl;

  std::cout << "Max fuel: " << parameters.get_refueling_amount() << std::endl;

  std::cout << "Base position:"
            << " lat " << instance.get_origin().get_lat()
            << " lon " << instance.get_origin().get_lon()
            << std::endl;

  idx num_served = 0;

  for(const auto& pair : pricer->get_path_variables())
  {
    const auto& path_variable = pair.second;

    double value = SCIPgetSolVal(scip, sol, path_variable->get_var());

    if(value < 0.5)
    {
      continue;
    }

    num_served += print_path(index++, path_variable->get_path());
  }

  const idx num_requests = instance.get_requests().size();

  if(num_served < num_requests)
  {
    std::cout << "Served "
              << num_served
              << " / "
              << num_requests
              << "( "
              << 100*(num_served / (double) num_requests)
              << "%)"
              << std::endl;
  }
  else
  {
    std::cout << "Served all requests, remaining gap: " << 100 * SCIPgetGap(scip) << "%" << std::endl;
  }

}

idx SchedulingProgram::print_path(idx index, const Path& path) const
{
  Units::Mass zero(0*Units::SI::kilogram);

  idx num_served = 0;

  const VertexMap<DateTime>& times = request_graph.get_times();
  VertexMap<Units::Mass> fuel(graph, zero);
  EdgeMap<Units::Mass> edge_burned_fuel(graph, Units::Mass(0*Units::SI::kilogram));

  BurnedFuel burned_fuel(request_graph.get_edge_requests(),
                         request_graph.get_edge_types());

  EdgeFuelDifference fuel_difference = request_graph.fuel_difference();

  const EdgeFunc<Units::Time>& travel_times = request_graph.get_travel_times();

  assert(path.connects(request_graph.get_origin(), request_graph.get_destination()));

  auto it = path.get_edges().rbegin();
  auto end = path.get_edges().rend();

  for(; it != end; ++it)
  {
    const Edge& edge = *it;

    Units::Mass current_fuel = fuel(edge.get_target());

    Units::Mass initial_fuel = fuel_difference(edge, current_fuel);

    fuel(edge.get_source()) = initial_fuel;

    edge_burned_fuel(edge) = burned_fuel(edge, initial_fuel, current_fuel);
  }

  const VertexMap<Point>& coordinates = request_graph.get_coordinates();
  const EdgeMap<OpType>& edge_types = request_graph.get_edge_types();
  const EdgeMap<Request>& edge_requests = request_graph.get_edge_requests();

  for(const Edge& edge : path.get_edges())
  {
    if(edge_types(edge) == OpType::BASE_WAITING)
    {
      continue;
    }

    std::cout << "Tanker " << index << " moves from"
              << " lat " << coordinates(edge.get_source()).get_lat()
              << " lon " << coordinates(edge.get_source()).get_lon()
              << " to"
              << " endlat " << coordinates(edge.get_target()).get_lat()
              << " endlon " << coordinates(edge.get_target()).get_lon();

    std::cout << ", leaves at " << ((times(edge.get_source()) - initial_time).total_seconds()) / 3600. << " [h]";
    std::cout << ", arrives at " << ((times(edge.get_target()) - initial_time).total_seconds()) / 3600. << " [h]";

    std::cout << ", fuel at origin: "
              << fuel(edge.get_source())
              << ", fuel at destination: "
              << fuel(edge.get_target())
              << ", ";

    std::cout << "burned fuel: " << edge_burned_fuel(edge) << ", ";

    if(edge_types(edge) == OpType::REFUELING)
    {
      const Request& request = edge_requests(edge);

      ++num_served;

      std::cout << "serving " << request.get_amount() << " to cruiser " << request.get_flight() << ", ";
    }

    Units::Length distance = coordinates(edge.get_source()).distance(coordinates(edge.get_target()));

    std::cout << "distance: " << distance << ", ";

    std::cout << "travel time: " << travel_times(edge) << std::endl;
  }

  return num_served;
}

void SchedulingProgram::create_combined_variables()
{
  const EdgeMap<OpType>& edge_types = request_graph.get_edge_types();

  for(const Edge& edge : graph.get_edges())
  {
    const OpType edge_type = edge_types(edge);

    if(edge_type != OpType::CLIMB &&
       edge_type != OpType::DESCENT &&
       edge_type != OpType::FLIGHT)
    {
      return;
    }

    std::ostringstream namebuf;
    SCIP_VAR* var;

    namebuf << "x_"
            << edge.get_source()
            << "_"
            << edge.get_target();

    SCIP_CALL_EXC(SCIPcreateVar(scip,
                                &var,
                                namebuf.str().c_str(),
                                0.0, 1.0,
                                0.0,  // cost
                                SCIP_VARTYPE_BINARY,
                                TRUE,
                                FALSE,
                                NULL, NULL, NULL, NULL, NULL));

    SCIP_CALL_EXC(SCIPchgVarBranchPriority(scip, var, 1));

    SCIP_CALL_EXC(SCIPaddVar(scip, var));

    combined_variables(edge) = var;
  }
}

void SchedulingProgram::create_covering_constraints()
{
  const RequestMap<Vertex>& origin_vertices = request_graph.get_origin_vertices();

  for(const Request& request : instance.get_requests())
  {
    std::ostringstream namebuf;
    SCIP_CONS* cons;

    namebuf << "covering_" << request.get_index();

    SCIP_CALL_EXC(SCIPcreateConsLinear(scip,
                                       &cons,
                                       namebuf.str().c_str(),
                                       0,
                                       NULL,
                                       NULL,
                                       1.,                 // lhs
                                       SCIPinfinity(scip), // rhs
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

    covering_constraints(origin_vertices(request)) = cons;
  }
}

const std::unordered_map<Path, std::shared_ptr<PathVariable>>&
SchedulingProgram::get_path_variables() const
{
  assert(pricer);

  return pricer->get_path_variables();
}

const EdgeFunc<std::vector<std::shared_ptr<PathVariable>>>&
SchedulingProgram::edge_path_variables() const
{
  return get_pricer().edge_path_variables();
}

DualValues
SchedulingProgram::get_dual_values(DualCostType cost_type,
                                   const std::vector<std::shared_ptr<PathVariable>>& entire_path_variables) const
{
  EdgeMap<double> edge_values(request_graph, 0.);

  const auto& covering_constraints = get_pricer().get_covering_constraints();

  for(const Request& request : request_graph.get_instance().get_requests())
  {
    const Edge& edge = request_graph.get_request_edges()(request);

    SCIP_CONS* cons = covering_constraints(edge.get_source());

    assert(SCIPconsIsTransformed(cons));

    double dual_value = cons_dual_value(scip, cons, cost_type);

    assert(SCIPisFeasGE(scip, dual_value, 0.));

    edge_values(edge) += dual_value;

    /*
    Log(debug) << "Dual value of covering constraint of "
               << edge.get_source()
               << "_"
               << edge.get_target()
               << ": "
               << dual_value;
    */
  }

  assert(pricer);

  // redistribute dual values away from the reduced costs
  // of the paths if necessary.

  if(pricer->get_costs().nonnegative() && cost_type == DualCostType::SIMPLE)
  {
    for(const auto& entire_path_variable : entire_path_variables)
    {
      const Path& path = entire_path_variable->get_path();
      SCIP_VAR* var = entire_path_variable->get_var();

      double reduced_cost = SCIPgetVarRedcost(scip, var);

      if(!SCIPisFeasNegative(scip, reduced_cost))
      {
        continue;
      }

      if(edge_pricer)
      {
        const auto& compound_variables = edge_pricer->get_compound_variables();

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
      }

      const auto& edge_types = request_graph.get_edge_types();

      for(const Edge& edge : path.get_edges())
      {
        if(edge_types(edge) == OpType::REFUELING)
        {
          double value = std::min(-reduced_cost,
                                  edge_values(edge));

          assert(SCIPisFeasGE(scip, value, 0.));

          edge_values(edge) -= value;
          reduced_cost += value;
        }

        if(SCIPisFeasZero(scip, reduced_cost))
        {
          break;
        }
      }

      assert(SCIPisFeasZero(scip, reduced_cost));
    }
  }

  if(edge_pricer)
  {
    edge_pricer->add_dual_values(edge_values, cost_type);
  }

  if(constraint_handler)
  {
    constraint_handler->add_dual_values(edge_values, cost_type);
  }

  auto path_constraint = get_pricer().get_path_constraint();

  if(path_constraint)
  {
    assert(SCIPconsIsTransformed(path_constraint));
  }

  const double path_cost = cons_dual_value(scip,
                                           path_constraint,
                                           cost_type);

  return DualValues(edge_values,
                    path_cost,
                    SCIPgetLPObjval(scip),
                    request_graph,
                    get_max_num_paths());
}

EdgeMap<double> SchedulingProgram::total_flow(SCIP_SOL* sol) const
{
  EdgeMap<double> flow(graph, 0.);

  for(const auto& pair : get_path_variables())
  {
    const auto& variable = pair.second;

    const double value = SCIPgetSolVal(getSCIP(), sol, variable->get_var());

    if(cmp::zero(value))
    {
      continue;
    }

    for(const auto& edge : variable->get_path().get_edges())
    {
      flow(edge) += value;
    }
  }

  return flow;
}

void SchedulingProgram::add_solution(const Solution& solution,
                                     SCIP_HEUR* heur)
{
  assert(pricer);

  std::vector<Path> paths;

  for(const auto& solution_path : solution.get_paths())
  {
    paths.push_back(request_graph.path_for(solution_path));
  }

  std::vector<std::shared_ptr<PathVariable>> solution_variables;

  for(const Path& path : paths)
  {
    solution_variables.push_back(pricer->add_path(path, false));
  }

  pricer->add_solution(solution_variables, heur);
}

bool SchedulingProgram::has_fixed_edges() const
{
  assert(edge_pricer);
  return edge_pricer->has_fixed_edges();
}
