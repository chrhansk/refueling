#include <fstream>

#include "log.hh"

#include "reader/instance_reader.hh"

#include "request_graph.hh"
#include "request_graph_fixture.hh"

TEST_F(RequestGraphFixture, DeliveredFuel)
{
  RequestGraph request_graph(instance);
  const Graph& graph = request_graph;

  for(const auto& request : instance.get_requests())
  {
    Edge request_edge = request_graph.get_request_edges()(request);

    EXPECT_EQ(request_graph.delivered_fuel(request_edge),
              request.get_amount());
  }

  for(const Edge& edge : graph.get_edges())
  {
    auto type = request_graph.get_edge_types()(edge);
    if(type != OpType::REFUELING)
    {
      EXPECT_EQ(request_graph.delivered_fuel(edge),
                Units::Mass(0*Units::SI::kilogram));
    }
  }
}

TEST_F(RequestGraphFixture, RequestPaths)
{
  RequestGraph request_graph(instance);

  for(const auto& request : instance.get_requests())
  {
    Edge request_edge = request_graph.get_request_edges()(request);

    Path request_path = request_graph.get_request_path(request);

    EXPECT_TRUE(request_path.contains(request_edge));

    EXPECT_TRUE(request_path.connects(request_graph.get_origin(),
                                      request_graph.get_destination()));
  }
}

TEST_F(RequestGraphFixture, Coordinates)
{
  RequestGraph request_graph(instance);

  const auto& coordinates = request_graph.get_coordinates();

  EXPECT_EQ(coordinates(request_graph.get_origin()),
            instance.get_origin());

  EXPECT_EQ(coordinates(request_graph.get_destination()),
            instance.get_origin());

  for(const auto& request : instance.get_requests())
  {
    Edge edge = request_graph.get_request_edges()(request);

    EXPECT_EQ(coordinates(edge.get_source()),
              request.get_origin());

    EXPECT_EQ(coordinates(edge.get_target()),
              request.get_destination());
  }
}

TEST_F(RequestGraphFixture, RequestEdges)
{
  RequestGraph request_graph(instance);

  for(const auto& request : instance.get_requests())
  {
    Edge edge = request_graph.get_request_edges()(request);
    Vertex source = request_graph.get_origin_vertices()(request);
    Vertex target = request_graph.get_destination_vertices()(request);

    EXPECT_EQ(edge.get_source(), source);
    EXPECT_EQ(edge.get_target(), target);
  }
}

TEST_F(RequestGraphFixture, TopologicalOrdering)
{
  RequestGraph request_graph(instance);
  const Graph& graph = request_graph;

  const auto& ordering = request_graph.get_topological_ordering();

  for(const Edge& edge : graph.get_edges())
  {
    auto source = std::find(std::begin(ordering),
                            std::end(ordering),
                            edge.get_source());

    auto target = std::find(std::begin(ordering),
                            std::end(ordering),
                            edge.get_target());

    EXPECT_NE(source, std::end(ordering));
    EXPECT_NE(target, std::end(ordering));

    EXPECT_TRUE(source < target);
  }
}

TEST_F(RequestGraphFixture, CentralPath)
{
  RequestGraph request_graph(instance);

  Path central_path = request_graph.get_central_path();

  EXPECT_TRUE(central_path.connects(request_graph.get_origin(),
                                    request_graph.get_destination()));

  for(const auto& request : instance.get_requests())
  {
    Edge request_edge = request_graph.get_request_edges()(request);

    EXPECT_FALSE(central_path.contains(request_edge));
  }
}
