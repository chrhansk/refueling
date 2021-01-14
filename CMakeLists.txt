cmake_minimum_required(VERSION 3.1)

project(refueling)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

enable_testing()

configure_file("${CMAKE_SOURCE_DIR}/cmake/CMakeLists.txt.in"
  googletest-download/CMakeLists.txt)

execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

execute_process(COMMAND ${CMAKE_COMMAND} --build .
  WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
  ${CMAKE_BINARY_DIR}/googletest-build)

find_package(SCIP REQUIRED)

include_directories(${SCIP_INCLUDE_DIRS})

find_package(Boost 1.54
  COMPONENTS date_time filesystem log log_setup program_options thread system
  REQUIRED)

add_definitions(-DBOOST_LOG_DYN_LINK)

include_directories(${BOOST_INCLUDE_DIRS})

if(CMAKE_BUILD_TYPE MATCHES "Release")
  message(STATUS "Release mode, disabling assertions")
  add_definitions("-DNDEBUG")
endif()

include_directories(${CMAKE_BINARY_DIR}/)

add_subdirectory(src)
