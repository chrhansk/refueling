#include "gtest/gtest.h"

#include "log.hh"

#include "program/request_pattern.hh"

#include "request_graph_fixture.hh"

TEST_F(RequestGraphFixture, SinglePaths)
{
  RequestGraph request_graph(instance);

  for(const auto& request : instance.get_requests())
  {
    Path request_path = request_graph.get_request_path(request);

    RequestPattern request_pattern(instance,
                                   request_graph.get_edge_types(),
                                   request_graph.get_edge_requests(),
                                   request_path);

    EXPECT_TRUE(request_pattern.contains(request));

    for(const auto& other : instance.get_requests())
    {
      if(other != request)
      {
        EXPECT_FALSE(request_pattern.contains(other));
      }
    }
  }

}
