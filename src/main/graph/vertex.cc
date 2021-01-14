#include "vertex.hh"

Vertex::Vertex()
  : index(std::numeric_limits<idx>::max())
{
}

idx Vertex::get_index() const
{
  return index;
}

bool Vertex::operator==(const Vertex& other) const
{
  return get_index() == other.get_index();
}

bool Vertex::operator!=(const Vertex& other) const
{
  return !(*this == other);
}

bool Vertex::operator<(const Vertex& other) const
{
  return get_index() < other.get_index();
}

bool Vertex::operator<=(const Vertex& other) const
{
  return get_index() <= other.get_index();
}

std::ostream& operator<<(std::ostream& out, const Vertex& vertex)
{
  out << vertex.get_index();
  return out;
}
