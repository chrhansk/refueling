#include "edge.hh"

#include "vertex_set.hh"

#include <cassert>

Vertex Edge::get_source() const
{
  return source;
}

Vertex Edge::get_target() const
{
  return target;
}

Vertex Edge::get_endpoint(Direction direction) const
{
  return (direction == Direction::OUTGOING)
    ? get_target()
    : get_source();
}

Vertex Edge::get_opposite(Vertex vertex) const
{
  if(vertex == get_source())
  {
    return get_target();
  }
  else
  {
    assert(vertex == get_target());
    return get_source();
  }
}

idx Edge::get_index() const
{
  return index;
}

bool Edge::operator==(const Edge& other) const
{
  return get_index() == other.get_index();
}

bool Edge::operator!=(const Edge& other) const
{
  return !(*this == other);
}

bool Edge::enters(const VertexSet& vertices) const
{
  return !vertices.contains(get_source()) && vertices.contains(get_target());
}

bool Edge::leaves(const VertexSet& vertices) const
{
  return vertices.contains(get_source()) && !vertices.contains(get_target());
}

bool Edge::intersects(const VertexSet& vertices) const
{
  return vertices.contains(get_source()) || vertices.contains(get_target());
}
