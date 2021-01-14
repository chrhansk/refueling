#ifndef SHORTEST_PATH_TREE_HH
#define SHORTEST_PATH_TREE_HH

#include <stdexcept>

#include "graph/graph.hh"
#include "graph/edge_map.hh"

#include "path/path.hh"

#include "label.hh"
#include "label_heap.hh"
#include "router.hh"

template <Direction direction = Direction::OUTGOING, class T = num>
class ShortestPathTree
{
private:
  const Graph& graph;
  const EdgeFunc<T>& costs;
  Vertex root;

  LabelHeap<Label<T>> heap;
  T max_dist;

  template<class Predicate, class Filter = AllEdgeFilter>
  void extend_while(Predicate predicate, Filter filter = Filter());

public:
  ShortestPathTree(const Graph& graph,
                   const EdgeFunc<T>& costs,
                   Vertex root);

  template<class Filter = AllEdgeFilter>
  void extend(Filter filter = Filter());

  template<class Filter = AllEdgeFilter>
  Vertex next(Filter filter = Filter());

  template<class Filter = AllEdgeFilter>
  void extend(Vertex vertex, Filter filter = Filter());

  template <class It, class Filter = AllEdgeFilter>
  void extend(It begin, It end, Filter filter = Filter());

  bool explored(Vertex vertex) const;

  T distance(Vertex vertex) const;

  bool done() const
  {
    return heap.is_empty();
  }

  T max_distance() const
  {
    return max_dist;
  }

  Path path(Vertex vertex) const;

};

template <Direction direction, class T>
ShortestPathTree<direction, T>::ShortestPathTree(const Graph& graph,
                                                 const EdgeFunc<T>& costs,
                                                 Vertex root)
  : graph(graph),
    costs(costs),
    root(root),
    heap(graph),
    max_dist(0)
{
  Label<T> rootLabel = Label<T>(root, Edge(), 0);

  heap.update(rootLabel);
}

template <Direction direction, class T>
template <class Predicate, class Filter>
void ShortestPathTree<direction, T>::extend_while(Predicate predicate, Filter filter)
{
  while(!heap.is_empty() && predicate())
  {
    next(filter);
  }
}

template <Direction direction, class T>
template <class Filter>
Vertex ShortestPathTree<direction, T>::next(Filter filter)
{
  assert(!done());

  const Label<T>& current = heap.extract_min();

  max_dist = std::max(max_dist, current.get_cost());

  for(const Edge& edge : graph.get_edges(current.get_vertex(), direction))
  {
    if(!filter(edge))
    {
      continue;
    }

    Label<T> nextLabel = Label<T>(edge.get_endpoint(direction),
                                  edge,
                                  current.get_cost() + costs(edge));

    heap.update(nextLabel);
  }

  return current.get_vertex();
}

template <Direction direction, class T>
template <class Filter>
void ShortestPathTree<direction, T>::extend(Filter filter)
{
  extend_while([] () -> bool {return true;}, filter);
}

template<Direction direction, class T>
template<class Filter>
void ShortestPathTree<direction, T>::extend(Vertex vertex, Filter filter)
{
  extend_while([&] () -> bool {return !explored(vertex);}, filter);
}

template <Direction direction, class T>
template <class It, class Filter>
void ShortestPathTree<direction, T>::extend(It begin, It end, Filter filter)
{
  for(auto it = begin; it != end; ++it)
  {
    extend_while([&] () -> bool {return !explored(*it);}, filter);
  }
}

template <Direction direction, class T>
bool ShortestPathTree<direction, T>::explored(Vertex vertex) const
{
  return heap.get_label(vertex).get_state() == State::SETTLED;
}

template <Direction direction, class T>
T ShortestPathTree<direction, T>::distance(Vertex vertex) const
{
  if(!explored(vertex))
  {
    throw std::invalid_argument("Vertex has not been explored");
  }

  return heap.get_label(vertex).get_cost();
}

template <Direction direction, class T>
Path ShortestPathTree<direction, T>::path(Vertex vertex) const
{
  if(!explored(vertex))
  {
    throw std::invalid_argument("Vertex has not been explored");
  }

  Path path;

  while(vertex != root)
  {
    Edge edge = heap.get_label(vertex).get_edge();

    path.add(edge, opposite(direction));

    vertex = edge.get_endpoint(opposite(direction));
  }

  return path;
}

#endif /* SHORTEST_PATH_TREE_HH */
