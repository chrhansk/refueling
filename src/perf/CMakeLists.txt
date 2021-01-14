add_library(perf_common
  EXCLUDE_FROM_ALL
  program/program_benchmark.cc)

target_link_libraries(perf_common
  common)

add_custom_target(build_benchmarks)

add_custom_target(run_benchmarks
  DEPENDS build_benchmarks)

add_custom_target(collect)

add_custom_target(collect_fuel)
add_custom_target(collect_count)

add_dependencies(collect collect_count)
add_dependencies(collect collect_fuel)

add_dependencies(collect build_benchmarks)

set(DATASET_DIR "${CMAKE_SOURCE_DIR}/dataset")

function(add_benchmark)
  cmake_parse_arguments(PARSE_ARGV 0 ARGS "" "TARGET" "SOURCES")

  add_executable(${ARGS_TARGET}
    EXCLUDE_FROM_ALL
    ${ARGS_SOURCES})

  target_include_directories(${ARGS_TARGET}
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}")

  add_dependencies(build_benchmarks ${ARGS_TARGET})

  target_link_libraries(${ARGS_TARGET} perf_common)
endfunction()

function(add_collect_target)
  cmake_parse_arguments(PARSE_ARGV 0 ARGS "PATH_COUNT" "INSTANCE" "DRIVER")

  set(INSTANCE_DIR "${DATASET_DIR}/${ARGS_INSTANCE}")

  if(${ARGS_PATH_COUNT})
    set(COLLECT_TARGET_NAME "collect_${ARGS_DRIVER}_${ARGS_INSTANCE}_count")

    set(COLLECT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${COLLECT_TARGET_NAME}.csv")

    add_custom_target("${COLLECT_TARGET_NAME}"
      COMMAND "${ARGS_DRIVER}" "--path_count" "${INSTANCE_DIR}" > "${COLLECT_FILE}")

    add_dependencies("${COLLECT_TARGET_NAME}" "${ARGS_DRIVER}")

    add_dependencies(collect_count "${COLLECT_TARGET_NAME}")
    add_dependencies("collect_${INSTANCE}" "${COLLECT_TARGET_NAME}")
    add_dependencies(collect "collect_${INSTANCE}")

  else()
    set(COLLECT_TARGET_NAME "collect_${ARGS_DRIVER}_${ARGS_INSTANCE}_fuel")

    set(COLLECT_FILE "${CMAKE_CURRENT_BINARY_DIR}/${COLLECT_TARGET_NAME}.csv")

    add_custom_target("${COLLECT_TARGET_NAME}"
      COMMAND "${ARGS_DRIVER}" "${INSTANCE_DIR}" > "${COLLECT_FILE}")

    add_dependencies("${COLLECT_TARGET_NAME}" "${ARGS_DRIVER}")

    add_dependencies(collect_fuel "${COLLECT_TARGET_NAME}")
    add_dependencies("collect_${INSTANCE}" "${COLLECT_TARGET_NAME}")
    add_dependencies(collect "collect_${INSTANCE}")

  endif()

endfunction()

add_benchmark(
  TARGET fundamental_path_benchmark
  SOURCES program/fundamental_path_program_benchmark.cc)

add_benchmark(
  TARGET scheduling_benchmark
  SOURCES program/scheduling_program_benchmark.cc)

set(INSTANCES
  artificial
  asia
  atlantic)

foreach(INSTANCE ${INSTANCES})
  add_custom_target("collect_${INSTANCE}")

  add_collect_target(
    INSTANCE ${INSTANCE}
    DRIVER fundamental_path_benchmark)

  add_collect_target(
    INSTANCE ${INSTANCE}
    PATH_COUNT
    DRIVER fundamental_path_benchmark)

  add_collect_target(
    INSTANCE ${INSTANCE}
    DRIVER scheduling_benchmark)

  add_collect_target(
    INSTANCE ${INSTANCE}
    PATH_COUNT
    DRIVER scheduling_benchmark)

endforeach()
