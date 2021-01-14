#include <fstream>

#include "gtest/gtest.h"

#include "defs.hh"
#include "log.hh"

#include "reader/instance_reader.hh"

TEST(InstanceReader, Read)
{
  log_init();

  InstanceReader reader;

  std::ostringstream namebuf;

  namebuf << DATASET_DIRECTORY;
  namebuf << "/atlantic/Tankerrequestfilebase3.dat";

  std::ifstream input(namebuf.str());

  Instance instance = reader.read(input);

  EXPECT_EQ(instance.get_requests().size(),
            1428);

  EXPECT_EQ(instance.get_origin(),
            Point(48.946133, -54.566044 ));
}
