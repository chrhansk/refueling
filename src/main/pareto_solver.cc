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

#include "pareto_front.hh"

#include "log.hh"

struct SolutionComparator
{
  bool operator()(const Solution& first, const Solution& second) const
  {
    return first.total_burned_fuel() < second.total_burned_fuel();
  }
};

int main(int argc, char *argv[])
{
  log_init();

  po::options_description desc("Allowed options");

  std::string input_name;
  int time_limit = -1;

  desc.add_options()
    ("help", "produce help message")
    ("time_limit", po::value<int>(&time_limit)->default_value(-1), "time limit")
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
    std::cerr << "Usage: "
              << argv[0]
              << " [options] <input> <ouput>"
              << std::endl;

    std::cerr << desc << std::endl;

    return 1;
  }

  po::notify(vm);

  InstanceReader reader;

  Log(info) << "Reading input file " << input_name;

  std::ifstream input(input_name);

  Instance instance = reader.read(input);

  Log(info) << "Successfully read an instance with " << instance.get_requests().size() << " requests";

  FundamentalPathGraph fundamental_path_graph(instance);

  const Parameters& parameters = fundamental_path_graph.get_parameters();

  Solution min_tanker_solution;
  Solution max_tanker_solution;

  {
    auto func = ZeroFunction();

    FundamentalPathProgram fundamental_path_program(fundamental_path_graph,
                                                    func);

    bool solved = fundamental_path_program.solve(time_limit);

    assert(solved);

    min_tanker_solution = fundamental_path_program.get_solution();
  }

  {
    auto burned_fuel = BurnedFuel(fundamental_path_graph.get_edge_requests(),
                                  fundamental_path_graph.get_edge_types());

    auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
      {
        return amount / Units::SI::kilogram;
      });

    FundamentalPathProgram fundamental_path_program(fundamental_path_graph,
                                                    fuel_cost,
                                                    instance.get_requests().size());

    bool solved = fundamental_path_program.solve(time_limit);

    assert(solved);

    max_tanker_solution = fundamental_path_program.get_solution();
  }

  const int min_num_tankers = min_tanker_solution.get_paths().size();
  const int max_num_tankers = max_tanker_solution.get_paths().size();

  assert(min_tanker_solution.is_valid(instance, parameters));
  assert(max_tanker_solution.is_valid(instance, parameters));

  ParetoFront<int, Solution, SolutionComparator> pareto_front(std::make_pair(min_num_tankers,
                                                                             min_tanker_solution),
                                                              std::make_pair(max_num_tankers,
                                                                             max_tanker_solution));

  for(int num_tankers = min_num_tankers + 1;
      num_tankers < max_num_tankers;
      ++num_tankers)
  {
    Solution current_solution;

    {
      auto burned_fuel = BurnedFuel(fundamental_path_graph.get_edge_requests(),
                                    fundamental_path_graph.get_edge_types());

      auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
        {
          return amount / Units::SI::kilogram;
        });

      FundamentalPathProgram fundamental_path_program(fundamental_path_graph,
                                                      fuel_cost,
                                                      num_tankers);

      auto lower_bound = pareto_front.lower_bound(num_tankers);

      if(lower_bound != pareto_front.get_entries().end())
      {
        fundamental_path_program.add_solution(lower_bound->second);
      }

      bool solved = fundamental_path_program.solve(time_limit);

      assert(solved);

      current_solution = fundamental_path_program.get_solution();

      assert(current_solution.is_valid(instance, parameters));
    }

    int current_num_tankers = current_solution.get_paths().size();

    pareto_front.insert(std::make_pair(current_num_tankers, current_solution));
  }

  for(const auto& entry : pareto_front.get_entries())
  {
    const Solution& solution = entry.second;
    int num_paths = entry.first;

    Log(info) << "Solution with " << num_paths
              << " tankers, burned fuel: "
              << solution.total_burned_fuel()
              << ", remaining gap: "
              << solution.get_gap();

    std::string output_name;

    {
      std::ostringstream namebuf;

      namebuf << input_name << "_solution_" << num_paths << ".txt";

      output_name = namebuf.str();
    }

    std::ofstream out(output_name);

    out << solution;

  }

  return 0;
}
