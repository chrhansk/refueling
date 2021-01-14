add_custom_target(build_tests)

set(TEST_COMMON_SRC
  request_graph_fixture.cc)

add_library(test_common ${TEST_COMMON_SRC})

function(add_unit_test TEST_NAME)
  get_filename_component(BASE_NAME ${TEST_NAME} NAME)
  add_executable(${BASE_NAME}
    EXCLUDE_FROM_ALL
    ${TEST_NAME}.cc)

  target_link_libraries(${BASE_NAME}
    test_common
    common
    gtest
    gtest_main)

  add_dependencies(build_tests ${BASE_NAME})

  add_test(NAME ${BASE_NAME} COMMAND ${BASE_NAME})
endfunction()

include_directories("${CMAKE_CURRENT_SOURCE_DIR}")

add_unit_test(bounded_router/bounded_label_set_test)
add_unit_test(fuel_difference_test)
add_unit_test(pareto_front_test)
add_unit_test(path/path_test)
add_unit_test(program/request_pattern_test)
add_unit_test(reader/instance_reader_test)
add_unit_test(request_graph_test)
