#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "reader/instance_reader.hh"

#include "bounded_router/bounded_router.hh"

#include "request_graph.hh"
#include "program/scheduling_program.hh"

#include "program/pricer/simple_path_pricer.hh"

#include "fuel_cost_function.hh"


#include "log.hh"

int main(int argc, char *argv[])
{
  log_init();

  po::options_description desc("Allowed options");

  std::string input_name;
  int max_num_paths = -1;
  int time_limit = -1;

  bool use_path_count = false;

  desc.add_options()
    ("help", "produce help message")
    ("path_count", po::bool_switch(&use_path_count), "minimize path count instead of fuel consumption")
    ("time_limit", po::value<int>(&time_limit)->default_value(-1), "time limit")
    ("max_num_paths", po::value<int>(&max_num_paths)->default_value(-1), "limit on the number of paths")
    ("input", po::value<std::string>(&input_name)->required(), "input file");

  po::variables_map vm;

  po::positional_options_description positional_options;
  positional_options.add("input", 1);

  po::store(po::command_line_parser(argc, argv)
            .options(desc)
            .positional(positional_options)
            .run(),
            vm);

  if(vm.count("help"))
  {
    std::cout << "Usage: "
              << argv[0]
              << " [options] <input>"
              << std::endl;

    std::cout << desc << std::endl;

    return 1;
  }

  po::notify(vm);

  InstanceReader reader;

  Log(info) << "Reading input file " << input_name;

  std::ifstream input(input_name);

  Instance instance = reader.read(input);

  Log(info) << "Successfully read an instance with " << instance.get_requests().size() << " requests";

  RequestGraph request_graph(instance);

  SchedulingProgram program(request_graph,
                            max_num_paths);

  bool use_artificial_variables = false;

  if(use_path_count)
  {
    Solution initial_solution;

    PathCount path_count(request_graph.get_origin());

    PathPricer* pricer = new SimplePathPricer(program, path_count, use_artificial_variables);

    bool solved = program.solve(pricer, time_limit);

    if(solved)
    {
      auto solution = program.get_solution();

      //std::cout << solution;
    }

  }
  else
  {
    BurnedFuel burned_fuel(request_graph.get_edge_requests(),
                           request_graph.get_edge_types());

    auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
      {
        return amount / Units::SI::kilogram;
      });

    PathPricer* pricer = new SimplePathPricer(program, fuel_cost, use_artificial_variables);

    program.solve(pricer, time_limit);
  }

  return 0;
}
