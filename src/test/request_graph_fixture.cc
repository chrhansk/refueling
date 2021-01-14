#include "request_graph_fixture.hh"

#include <fstream>

#include "reader/instance_reader.hh"

void RequestGraphFixture::SetUp()
{
  InstanceReader reader;

  std::ostringstream namebuf;

  namebuf << DATASET_DIRECTORY;
  namebuf << "/atlantic/Tankerrequestfilebase7.dat";

  std::ifstream input(namebuf.str());

  instance = reader.read(input);
}
