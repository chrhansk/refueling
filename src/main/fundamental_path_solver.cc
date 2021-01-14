#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "reader/instance_reader.hh"

#include "bounded_router/bounded_router.hh"

#include "fundamental_path/fundamental_path_graph.hh"
#include "fundamental_path/fundamental_path_program.hh"

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
              << " [options] <input> <ouput>"
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

  FundamentalPathGraph fundamental_path_graph(instance);

  if(use_path_count)
  {
    auto func = ZeroFunction();

    FundamentalPathProgram program(fundamental_path_graph,
                                   func,
                                   max_num_paths);

    program.solve();

    Solution solution = program.get_solution();

    std::cout << solution;
  }
  else
  {
    if(max_num_paths == -1)
    {
      max_num_paths = instance.get_requests().size();
    }

    BurnedFuel burned_fuel(fundamental_path_graph.get_edge_requests(),
                           fundamental_path_graph.get_edge_types());

    auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
      {
        return amount / Units::SI::kilogram;
      });

    FundamentalPathProgram program(fundamental_path_graph,
                                   fuel_cost,
                                   max_num_paths);

    program.solve();

    Solution solution = program.get_solution();

    std::cout << solution;
  }

  return 0;
}
