#ifndef REQUEST_GRAPH_FIXTURE_HH
#define REQUEST_GRAPH_FIXTURE_HH

#include <gtest/gtest.h>

#include "defs.hh"

#include "instance.hh"

class RequestGraphFixture : public ::testing::Test
{
protected:
  void SetUp() override;

  Instance instance;
};


#endif /* REQUEST_GRAPH_FIXTURE_HH */
