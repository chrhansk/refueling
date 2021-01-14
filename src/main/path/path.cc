#include "path.hh"

#include <cassert>

#include <unordered_set>

#include "graph/vertex_set.hh"

Path::Path(std::initializer_list<Edge> edges)
{
  for(auto it = edges.begin(); it != edges.end(); ++it)
  {
    append(*it);
  }
}

void Path::clear()
{
  edges.clear();
}

void Path::append(const Edge& edge)
{
  if(!edges.empty())
  {
    assert(get_target() == edge.get_source());
  }

  edges.push_back(edge);
}

void Path::prepend(const Edge& edge)
{
  if(!edges.empty())
  {
    assert(get_source() == edge.get_target());
  }

  edges.push_front(edge);
}

void Path::append(const Path& path)
{
  for(const Edge& edge : path.get_edges())
  {
    append(edge);
  }
}

void Path::prepend(const Path& path)
{
  auto it = path.get_edges().rbegin();
  auto end = path.get_edges().rend();

  for(; it != end; ++it)
  {
    prepend(*it);
  }
}

void Path::pop_front()
{
  edges.pop_front();
}

void Path::pop_back()
{
  edges.pop_back();
}

const Path::Edges& Path::get_edges() const
{
  return edges;
}

void Path::add(const Edge& edge, Direction direction)
{
  if(direction == Direction::OUTGOING)
  {
    append(edge);
  }
  else
  {
    prepend(edge);
  }
}

bool Path::connects(Vertex source, Vertex target) const
{
  if(get_edges().empty())
  {
    return source == target;
  }

  return get_edges().begin()->get_source() == source and
    get_edges().rbegin()->get_target() == target;
}

bool Path::connects(Vertex source,
                    Vertex target,
                    Direction direction) const
{
  if(direction == Direction::OUTGOING)
  {
    return connects(source, target);
  }
  else
  {
    return connects(target, source);
  }
}

bool Path::contains(Vertex vertex) const
{
  for(const Edge& edge : get_edges())
  {
    if(edge.get_source() == vertex)
    {
      return true;
    }
  }

  return get_edges().rbegin()->get_target() == vertex;
}

bool Path::contains(Edge edge) const
{
  return std::find(std::begin(get_edges()),
                   std::end(get_edges()),
                   edge) != std::end(get_edges());
}

bool Path::is_simple() const
{
  if(get_edges().empty())
  {
    return true;
  }

  std::unordered_set<Vertex> vertices;

  for(const Edge& edge : get_edges())
  {
    if(vertices.find(edge.get_source()) != vertices.end())
    {
      return false;
    }
  }

  return vertices.find(get_edges().rbegin()->get_target())
    == vertices.end();
}

Vertex Path::get_source() const
{
  assert(!get_edges().empty());

  return get_edges().begin()->get_source();
}

Vertex Path::get_target() const
{
  assert(!get_edges().empty());

  return get_edges().rbegin()->get_target();
}

bool Path::is_tour(const Graph& graph) const
{
  if(get_edges().size() != graph.get_vertices().size())
  {
    return false;
  }

  VertexSet vertices(graph);

  for(const Edge& edge : get_edges())
  {
    if(vertices.contains(edge.get_source()))
    {
      return false;
    }

    vertices.insert(edge.get_source());
  }

  return true;
}

Vertex Path::get_endpoint(Direction direction) const
{
  assert(!get_edges().empty());

  if(direction == Direction::OUTGOING)
  {
    return get_target();
  }
  else
  {
    return get_source();
  }
}

std::ostream& operator<<(std::ostream& os, const Path& path)
{
  os << path.get_source();

  for(const Edge& edge : path.get_edges())
  {
    os << ", " << edge.get_target();
  }

  return os;
}
