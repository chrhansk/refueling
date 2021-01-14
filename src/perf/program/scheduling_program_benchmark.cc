#include "program_benchmark.hh"

#include "log.hh"

#include "request_graph.hh"
#include "program/scheduling_program.hh"

#include "program/pricer/simple_path_pricer.hh"

class SchedulingProgramBenchmark : public ProgramBenchmark
{
protected:
  virtual SolutionStats execute(const Instance& instance,
                                int time_limit,
                                bool use_path_count) override
  {
    RequestGraph request_graph(instance);

    bool use_artificial_variables = false;

    if(use_path_count)
    {
      SchedulingProgram program(request_graph,
                                -1,
                                Program::Settings().collect_stats(true));

      PathCount path_count(request_graph.get_origin());

      PathPricer* pricer = new SimplePathPricer(program,
                                                path_count,
                                                use_artificial_variables);

      program.solve(pricer, time_limit);

      return program.get_stats();
    }
    else
    {
      SchedulingProgram program(request_graph,
                                -1,
                                Program::Settings().collect_stats(true));

      BurnedFuel burned_fuel(request_graph.get_edge_requests(),
                             request_graph.get_edge_types());

      auto fuel_cost = burned_fuel.then<double>([](const Units::Mass& amount) -> double
        {
          return amount / Units::SI::kilogram;
        });

      PathPricer* pricer = new SimplePathPricer(program,
                                                fuel_cost,
                                                use_artificial_variables);

      program.solve(pricer, time_limit);

      return program.get_stats();
    }
  }
};

int main(int argc, char *argv[])
{
  SchedulingProgramBenchmark benchmark;

  benchmark.init(argc, argv);

  benchmark.execute_all();

  return 0;
}
