#include "gtest/gtest.h"

#include <unordered_map>

#include "log.hh"

#include "graph/graph.hh"
#include "path/path.hh"

TEST(Path, Map)
{
  Graph graph;

  Vertex source = graph.add_vertex();
  Vertex target = graph.add_vertex();

  Edge edge = graph.add_edge(source, target);

  std::unordered_map<Path, int> values;

  Path first{edge};
  Path second{edge};

  values.insert(std::make_pair(first, 0));
  values.insert(std::make_pair(second, 1));

  ASSERT_EQ(values.size(), 1);
}
