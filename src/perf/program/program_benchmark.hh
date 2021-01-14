#ifndef PROGRAM_BENCHMARK_HH
#define PROGRAM_BENCHMARK_HH

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

#include "instance.hh"
#include "program/program.hh"

class ProgramBenchmark
{
private:
  int time_limit;
  bool use_path_count;
  std::string input_name;

public:
  ProgramBenchmark();

  void execute_all();

  void init(int argc, char *argv[]);

protected:
  virtual SolutionStats execute(const Instance& instance,
                                int time_limit = -1,
                                bool use_path_count = false) = 0;
};


#endif /* PROGRAM_BENCHMARK_HH */
