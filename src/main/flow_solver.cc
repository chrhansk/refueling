#include <iostream>
#include <fstream>
#include <string>

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include "reader/instance_reader.hh"

#include "flow/flow_graph.hh"
#include "flow/flow_program.hh"

#include "log.hh"

int main(int argc, char *argv[])
{
  log_init();

  po::options_description desc("Allowed options");

  std::string input_name;

  desc.add_options()
    ("help", "produce help message")
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

  FlowGraph flow_graph(instance);

  FlowProgram flow_program(flow_graph);

  flow_program.solve();

  return 0;
}
