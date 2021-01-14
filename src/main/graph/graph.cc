#include "graph.hh"

#include <cassert>
#include <stack>

#include "subgraph.hh"
#include "vertex_set.hh"

Graph::Graph(idx size, const std::vector<Edge>& edges)
  : size(size)
{
  for(idx i = 0; i < size; ++i)
  {
    outgoing.push_back(std::vector<Edge>());
    incoming.push_back(std::vector<Edge>());
  }

  for(const Edge& edge : edges)
  {
    add_edge(edge);
  }

  //assert(check());
}

Graph::Graph(idx size, std::initializer_list<std::pair<idx, idx>> edges)
  : size(size)
{
  for(idx i = 0; i < size; ++i)
  {
    outgoing.push_back(std::vector<Edge>());
    incoming.push_back(std::vector<Edge>());
  }

  idx i = 0;
  for(const auto& pair : edges)
  {
    add_edge(Edge(Vertex(pair.first), Vertex(pair.second), i++));
  }
}

Vertex Graph::add_vertex()
{
  Vertex vertex(size);
  outgoing.push_back({});
  incoming.push_back({});
  ++size;
  return vertex;
}

void Graph::add_edge(const Edge& edge)
{
  assert(edge.get_index() == edges.size());
  assert(edge.get_source().get_index() < size);
  assert(edge.get_target().get_index() < size);

  outgoing[edge.get_source().get_index()].push_back(edge);
  incoming[edge.get_target().get_index()].push_back(edge);

  edges.push_back(Edge(edge));
}

const std::vector<Edge>& Graph::get_edges() const
{
  return edges;
}

Vertices Graph::get_vertices() const
{
  return Vertices(size);
}

const std::vector<Edge>& Graph::get_outgoing(Vertex vertex) const
{
  return outgoing[vertex.get_index()];
}

const std::vector<Edge>& Graph::get_incoming(Vertex vertex) const
{
  return incoming[vertex.get_index()];
}

const std::vector<Edge>& Graph::get_edges(Vertex vertex,
                                         Direction direction) const
{
  return (direction == Direction::OUTGOING) ?
    get_outgoing(vertex) :
    get_incoming(vertex);
}


bool Graph::are_adjacent(Vertex first,
                         Vertex second) const
{
  Edge edge;

  return has_edge(first, second, edge) || has_edge(second, first, edge);
}

bool Graph::has_edge(Vertex source,
                     Vertex target,
                     Edge& edge) const
{
  for(const Edge& out_edge : get_outgoing(source))
  {
    if(out_edge.get_target() == target)
    {
      edge = out_edge;
      return true;
    }
  }

  return false;
}

bool Graph::contains(const Edge& edge) const
{
  for(const Edge& outEdge : get_outgoing(edge.get_source()))
  {
    if(outEdge == edge)
    {
      return true;
    }
  }

  return false;
}

Edge Graph::add_edge(Vertex source, Vertex target)
{
  assert((size_t) source.get_index() < get_vertices().size());
  assert((size_t) target.get_index() < get_vertices().size());
  assert(source != target);

  Edge edge(source, target, edges.size());
  edges.push_back(edge);
  outgoing[source.get_index()].push_back(edge);
  incoming[target.get_index()].push_back(edge);

// const const  assert(check());

  return edge;
}

bool Graph::check() const
{
  for(const Vertex& vertex : get_vertices())
  {
    for(const Edge& edge : get_outgoing(vertex))
    {
      assert(edge.get_source() == vertex);
    }

    for(const Edge& edge : get_incoming(vertex))
    {
      assert(edge.get_target() == vertex);
    }
  }

  for(uint j = 0; j < get_edges().size(); ++j)
  {
    const Edge& edge = get_edges()[j];
    assert(edge.get_index() == j);

    const Vertex& source = edge.get_source();
    const Vertex& target = edge.get_target();

    bool found = false;

    for(const Edge& outEdge : get_outgoing(source))
    {
      if(outEdge == edge)
      {
        assert(outEdge.get_source() == source);
        assert(outEdge.get_target() == target);
        assert(!found);
        found = true;
      }
    }

    assert(found);
    found = false;

    for(const Edge& inEdge : get_incoming(target))
    {
      if(inEdge == edge)
      {
        assert(inEdge.get_source() == source);
        assert(inEdge.get_target() == target);
        assert(!found);
        found = true;
      }
    }

    assert(found);
  }

  return true;
}

VertexSet Graph::reachable(const Vertex& source) const
{
  VertexSet reachableVertices(*this);
  reachableVertices.insert(source);

  std::stack<Vertex> vertices;
  vertices.push(source);

  while(!vertices.empty())
  {
    Vertex current = vertices.top();
    vertices.pop();

    for(const Edge& edge : get_outgoing(current))
    {
      Vertex target = edge.get_target();
      if(!reachableVertices.contains(target))
      {
        reachableVertices.insert(target);
        vertices.push(target);
      }
    }
  }

  return reachableVertices;
}


SubGraph Graph::induced_subgraph(const std::vector<Vertex>& vertices) const
{
  SubGraph subGraph(*this, vertices);

  std::vector<Vertex> subVertices = ((const Graph&) subGraph).get_vertices().collect();

  for(idx i = 0; i < vertices.size(); ++i)
  {
    const Vertex& source = vertices[i];

    for(const Edge& edge : get_outgoing(source))
    {
      for(idx j = 0; j < vertices.size(); ++j)
      {
        const Vertex& target = vertices[j];

        if(source == target)
        {
          continue;
        }

        if(edge.get_target() == target)
        {
          subGraph.add_edge(subVertices[i], subVertices[j]);
        }
      }
    }
  }

  return subGraph;
}

SubGraph Graph::reachable_subgraph(const std::vector<Vertex>& vertices) const
{
  SubGraph subGraph(*this, vertices);

  std::vector<Vertex> subVertices = ((const Graph&) subGraph).get_vertices().collect();

  for(idx i = 0; i < vertices.size(); ++i)
  {
    const Vertex& source = vertices[i];

    VertexSet reachableVertices = reachable(source);

    for(idx j = 0; j < vertices.size(); ++j)
    {
      const Vertex& target = vertices[j];

      if(source == target)
      {
        continue;
      }

      if(reachableVertices.contains(target))
      {
        subGraph.add_edge(subVertices[i], subVertices[j]);
      }
    }
  }

  return subGraph;
}


Graph Graph::complete(idx numVertices)
{
  Graph graph(numVertices, {});

  for(const Vertex& source : graph.get_vertices())
  {
    for(const Vertex& target : graph.get_vertices())
    {
      if(source == target)
      {
        continue;
      }

      graph.add_edge(source, target);
    }
  }

  return graph;
}
