#include "program_benchmark.hh"

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "reader/instance_reader.hh"

#include "timer.hh"

const int TIME_LIMIT = 3600;

ProgramBenchmark::ProgramBenchmark()
  : time_limit(-1),
    use_path_count(false),
    input_name("")
{}

void ProgramBenchmark::init(int argc, char *argv[])
{
  log_init();

  po::options_description desc("Allowed options");

  desc.add_options()
    ("help", "produce help message")
    ("path_count", po::bool_switch(&use_path_count), "minimize path count instead of fuel consumption")
    ("time_limit", po::value<int>(&time_limit)->default_value(TIME_LIMIT), "time limit")
    ("input", po::value<std::string>(&input_name)->required(), "input file");

  po::variables_map vm;

  po::positional_options_description positional_options;
  positional_options.add("input", 1);

  po::store(po::command_line_parser(argc, argv)
            .options(desc)
            .positional(positional_options)
            .run(),
            vm);

  po::notify(vm);
}

void ProgramBenchmark::execute_all()
{

  std::cout << "Name;Size;Time;primalBound;dualBound;gap;"
            << "maxDepth;numIterations;numNodes;numVariables;"
            << "numConstraints;LPRootObjVal;rootLPTime;rootLPSolved;minRows;maxRows;avgRows;"
            << "minCols;maxCols;avgCols;numLPs;numCuts;separationTime"
            << std::endl;

  std::vector<fs::path> instances;

  fs::path instance_path(input_name);

  for(const auto& instance : fs::directory_iterator(instance_path))
  {
    if(!fs::is_regular_file(instance))
    {
      continue;
    }

    auto instance_path = instance.path();

    if(instance_path.extension() != ".dat")
    {
      continue;
    }

    instances.push_back(instance_path);
  }

  std::sort(std::begin(instances), std::end(instances));

  int time_limit = 3600;

  for(const auto& path : instances)
  {
    std::ifstream input(path.string());

    Instance instance = InstanceReader().read(input);

    Timer timer;

    SolutionStats stats = execute(instance,
                                  time_limit,
                                  use_path_count);

    double elapsed = timer.elapsed();

    elapsed = std::min(elapsed, (double) time_limit);

    std::cout << path.filename()
              << ";"
              << instance.get_requests().size()
              << ";"
              << elapsed
              << ";"
              << stats.primal_bound
              << ";"
              << stats.dual_bound
              << ";"
              << stats.gap
              << ";"
              << stats.max_depth
              << ";"
              << stats.num_iterations
              << ";"
              << stats.num_nodes
              << ";"
              << stats.num_variables
              << ";"
              << stats.num_constraints
              << ";"
              << stats.lp_stats.root_obj_val
              << ";"
              << stats.lp_stats.root_time
              << ";"
              << stats.lp_stats.root_lp_solved
              << ";"
              << stats.lp_stats.min_rows
              << ";"
              << stats.lp_stats.max_rows
              << ";"
              << stats.lp_stats.avg_rows
              << ";"
              << stats.lp_stats.min_cols
              << ";"
              << stats.lp_stats.max_cols
              << ";"
              << stats.lp_stats.avg_cols
              << ";"
              << stats.lp_stats.num_lps
              << ";"
              << stats.num_cuts
              << ";"
              << stats.separation_time
              << ";"
              << std::endl;
  }
}
