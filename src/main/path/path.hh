#ifndef PATH_HH
#define PATH_HH

#include <deque>
#include <memory>

#include "graph/edge.hh"
#include "graph/edge_map.hh"

/**
 * A Path given by a tuple of Edge%s. Each successive
 * pair of Edge%s of the Path shares a common Vertex.
 * A Path may contain cylces.
 **/
class Path
{
public:

  struct PathVertexIterator
  {
    std::deque<Edge>::const_iterator it;
    bool initial;

    Vertex operator*()
    {
      return initial ? it->get_source() : it->get_target();
    }

    const PathVertexIterator& operator++()
    {
      if(initial)
      {
        initial = false;
      }
      else
      {
        ++it;
      }

      return *this;
    }

    const bool operator!=(const PathVertexIterator& other)
    {
      return (it != other.it) || (initial != other.initial);
    }
  };

  struct PathVertices
  {
    const Path& path;

    PathVertexIterator begin() const
    {
      return PathVertexIterator{path.get_edges().begin(), true};
    }

    PathVertexIterator end() const
    {
      return PathVertexIterator{path.get_edges().end(), false};
    }

    std::vector<Vertex> collect() const
    {
      std::vector<Vertex> vertices;
      for(auto it = begin(); it != end(); ++it)
      {
        vertices.push_back(*it);
      }

      return vertices;
    }

    idx size() const
    {
      return path.get_edges().size();
    }
  };

public:
  typedef std::deque<Edge> Edges;

  typedef Edges::const_iterator EdgeIterator;
  typedef Edges::const_reverse_iterator ReverseEdgeIterator;

private:
  Edges edges;
public:

  /**
   * Constructs an empty Path.
   **/
  Path() {}

  /**
   * Constructs a path from the given Edges%s.
   **/
  Path(std::initializer_list<Edge> edges);

  void clear();

  /**
   * Appends an Edge to the given Path.
   **/
  void append(const Edge& edge);

  /**
   * Prepends an Edge to the given Path.
   **/
  void prepend(const Edge& edge);

  /**
   * Appends a Path to the given Path.
   **/
  void append(const Path& path);

  /**
   * Prepends a Path to the given Path.
   **/
  void prepend(const Path& path);

  void pop_front();

  void pop_back();

  /**
   * Returns the cost of the path with respect to
   * the given cost function.
   *
   * @tparam A cost function given as a map from Edge%s to numbers.
   *
   **/
  template <class T>
  num cost(const EdgeFunc<T>& func) const;

  /**
   * Returns the Edge%s in this Path.
   **/
  const Edges& get_edges() const;

  /**
   * Adds an Edge to this Path from the given Direction. An
   * INCOMING Edge is prepended, an OUTGOING Edge is appended.
   **/
  void add(const Edge& edge, Direction direction);

  /**
   * Returns whether this Path connects the given vertices.
   **/
  bool connects(Vertex source, Vertex target) const;

  /**
   * Returns whether this Path connects the given vertices in the given
   * Direction.
   **/
  bool connects(Vertex source, Vertex target, Direction direction) const;

  /**
   * Returns whether this Path contains the given Vertex.
   **/
  bool contains(Vertex vertex) const;

  /**
   * Returns whether this Path contains the given Edge.
   **/
  bool contains(Edge edge) const;

  /**
   * Returns whether this Path is simple, i.e. whether it does
   * not contains cycles.
   **/
  bool is_simple() const;

  /**
   * Returns the source Vertex of this Path.
   **/
  Vertex get_source() const;

  /**
   * Returns the target Vertex of this Path.
   **/
  Vertex get_target() const;

  bool is_tour(const Graph& graph) const;

  /**
   * Returns the endpoint of this Path with respect to
   * the given Direction-
   **/
  Vertex get_endpoint(Direction direction) const;

  /**
   * Returns whether this Path satisfies the given filter.
   * @tparam Filter A filer given by a map from Edge%s to boolean values.
   **/
  template <class Filter>
  bool satisfies(const Filter& filter) const;

  /**
   * Returns whether this Path decomposes into two subpaths such
   * that the subpath containing the source satisfies the given
   * forward filter and the one containing the target satisfies
   * the backward filter.
   *
   * @tparam ForwardFilter  A filer given by a map from Edge%s to boolean values.
   * @tparam BackwardFilter A filer given by a map from Edge%s to boolean values.
   **/
  template <class ForwardFilter, class BackwardFilter>
  bool satisfies(const ForwardFilter& forward_filter,
                 const BackwardFilter& backward_filter) const;

  /**
   * Returns whether this path contains any Edge%s.
   **/
  operator bool() const
  {
    return !(get_edges().empty());
  }

  PathVertices get_vertices() const
  {
    return PathVertices{*this};
  }

  bool operator==(const Path& other) const
  {
    return get_edges() == other.get_edges();
  }
};

template <class T>
num Path::cost(const EdgeFunc<T>& func) const
{
  num s = 0;

  for(const Edge& edge : get_edges())
  {
    s += func(edge);
  }

  return s;
}

template <class Filter>
bool Path::satisfies(const Filter& filter) const
{
  for(const Edge& edge : get_edges())
  {
    if(!filter(edge))
    {
      return false;
    }
  }

  return true;
}

template <class ForwardFilter, class BackwardFilter>
bool Path::satisfies(const ForwardFilter& forward_filter,
                     const BackwardFilter& backward_filter) const
{
  bool forward = true;

  for(const Edge& edge : get_edges())
  {
    if(forward)
    {
      if(!forward_filter(edge))
      {
        if(!backward_filter(edge))
        {
          return false;
        }

        forward = false;
      }
    }
    else if(!backward_filter(edge))
    {
      return false;
    }
  }

  return true;
}

std::ostream& operator<<(std::ostream& os, const Path& path);

namespace std
{
  /**
   * A hash function for edges
   */
  template<> struct hash<Path>
  {
    typedef Path argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type const& path) const
    {
      result_type seed = 0;

      for(const Edge& edge : path.get_edges())
      {
        hash_combination(seed, edge);
      }

      return seed;
    }
  };
}

#endif /* PATH_HH */
