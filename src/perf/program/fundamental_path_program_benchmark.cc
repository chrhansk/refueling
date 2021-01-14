#include "program_benchmark.hh"

#include "log.hh"

#include "fundamental_path/fundamental_path_graph.hh"
#include "fundamental_path/fundamental_path_program.hh"

class FundamentalPathProgramBenchmark : public ProgramBenchmark
{
protected:
  virtual SolutionStats execute(const Instance& instance,
                                int time_limit,
                                bool use_path_count) override
  {
    FundamentalPathGraph fundamental_path_graph(instance);

    if(use_path_count)
    {

      auto func = ZeroFunction();

      FundamentalPathProgram program(fundamental_path_graph,
                                     func,
                                     -1,
                                     Program::Settings().collect_stats(true));

      program.solve(time_limit);

      return program.get_stats();
    }
    else
    {
      int max_num_paths = instance.get_requests().size();

      BurnedFuel burned_fuel(fundamental_path_graph.get_edge_requests(),
                             fundamental_path_graph.get_edge_types());

      auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
        {
          return amount / Units::SI::kilogram;
        });

      FundamentalPathProgram program(fundamental_path_graph,
                                     fuel_cost,
                                     max_num_paths,
                                     Program::Settings().collect_stats(true));

      program.solve(time_limit);

      return program.get_stats();
    }
  }
};


int main(int argc, char *argv[])
{
  FundamentalPathProgramBenchmark benchmark;

  benchmark.init(argc, argv);

  benchmark.execute_all();

  return 0;
}
