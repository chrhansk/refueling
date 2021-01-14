set(COMMON_SRC
  ansi_color.cc
  cmp.cc
  edge_fuel_difference.cc
  fuel_cost_function.cc
  fuel_difference.cc
  log.cc
  point.cc
  parameters.cc
  request.cc
  request_graph.cc
  solution.cc
  time.cc
  util.cc
  bounded_router/cost_bound.cc
  bounded_router/bounded_label.cc
  bounded_router/bounded_label_set.cc
  bounded_router/bounded_router.cc
  bounded_router/forbidden_path_router.cc
  flow/flow_graph.cc
  flow/flow_program.cc
  graph/edge.cc
  graph/edge_set.cc
  graph/graph.cc
  graph/subgraph.cc
  graph/vertex.cc
  graph/vertex_set.cc
  path/path.cc
  fundamental_path/fundamental_path_graph.cc
  fundamental_path/fundamental_path_program.cc
  fundamental_path/pricer/fundamental_edge_pricer.cc
  fundamental_path/pricer/fundamental_path_pricer.cc
  fundamental_path/pricer/simple_fundamental_path_pricer.cc
  program/capacity_constraint_handler.cc
  program/dual_values.cc
  program/lp_observer.cc
  program/program.cc
  program/request_pattern.cc
  program/reduced_costs.cc
  program/scip_utils.cc
  program/scheduling_program.cc
  program/solution_stats.cc
  program/pricer/path_pricer.cc
  program/pricer/simple_path_pricer.cc
  program/pricer/edge_pricer.cc
  program/heuristics/covering_heuristic.cc
  program/heuristics/greedy_covering.cc
  program/heuristics/iterative_covering_heuristic.cc
  reader/instance_reader.cc
  router/max_clique.cc
  router/odd_cycle.cc
  router/router.cc
  writer/instance_writer.cc)

add_library(common ${COMMON_SRC})

configure_file(defs.hh.in ${CMAKE_BINARY_DIR}/defs.hh)

set(LIBS
  ${CMAKE_THREAD_LIBS_INIT}
  ${Boost_LIBRARIES}
  ${SCIP_LIBRARIES})

target_link_libraries(common ${LIBS})

add_executable(request_solver request_solver.cc)
target_link_libraries(request_solver common)

add_executable(fundamental_path_solver fundamental_path_solver.cc)
target_link_libraries(fundamental_path_solver common)

add_executable(pareto_solver pareto_solver.cc)
target_link_libraries(pareto_solver common)

add_executable(flow_solver flow_solver.cc)
target_link_libraries(flow_solver common)
