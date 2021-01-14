#ifndef ROUTER_HH
#define ROUTER_HH

#include "graph/edge_map.hh"
#include "graph/graph.hh"
#include "path/path.hh"

#include "label.hh"
#include "label_heap.hh"

class AllEdgeFilter
{
public:
  constexpr bool operator()(const Edge& edge) const
  {
    return true;
  }
};

class NoEdgeFilter
{
public:
  constexpr bool operator()(const Edge& edge) const
  {
    return false;
  }
};

/**
 * A SearchResult represents the result of a shortest Path
 * search. The search can be successful (a Path was found)
 * or unsuccessful (no Path was found). If the search
 * was successful, the SearchResult contains the corresponding
 * path.
 **/
template <class T = num>
class SearchResult
{

public:
  SearchResult(int settled, int labeled, bool found, Path path, T cost)
    : settled(settled),
      labeled(labeled),
      found(found),
      path(path),
      cost(cost)
  {}

  SearchResult()
    : settled(0),
      labeled(0),
      found(false),
      cost(0)
  {}
  SearchResult(const SearchResult& other) = default;

  static SearchResult not_found(idx settled, idx labeled)
  {
    return SearchResult(settled, labeled, false, Path(), 0);
  }

  int settled;
  int labeled;
  bool found;
  Path path;
  T cost;
};

/**
 * A base class for all routers.
 **/
template<class T = num>
class Router
{
public:
  /**
   * Finds the shortest path between the given vertices
   * with respect to the given (non-negative) costs.
   *
   * @param source The source Vertex
   * @param target The target Vertex
   * @param costs  A function containing (non-negative) costs.
   *
   * @return A SearchResult
   **/
  virtual SearchResult<T> shortest_path(Vertex source,
                                        Vertex target,
                                        const EdgeFunc<T>& costs) = 0;

  /**
   * Finds the shortest path between the given vertices
   * with respect to the given (non-negative) costs.
   *
   * @param source The source Vertex
   * @param target The target Vertex
   * @param costs  A function containing (non-negative) costs.
   * @param bound  An upper bound on the path cost. If a path
   *               is returned it is guaranteed to have a cost
   *               of at most the given bound value. If it is
   *               determined that no path satisfiying the given
   *               bound exists, then the computation can be
   *               aborted early.
   *
   * @return A SearchResult
   **/
  virtual SearchResult<T> shortest_path(Vertex source,
                                        Vertex target,
                                        const EdgeFunc<T>& costs,
                                        T bound) = 0;

  virtual ~Router() {}
};

/**
 * A class which find shortest paths by performing a unidirectional
 * search from the source Vertex.
 **/
template<class T = num>
class Dijkstra : public Router<T>
{
private:
  const Graph& graph;

public:
  Dijkstra(const Graph& graph)
    : graph(graph) {}

  SearchResult<T> shortest_path(Vertex source,
                                Vertex target,
                                const EdgeFunc<T>& costs) override;

  SearchResult<T> shortest_path(Vertex source,
                                Vertex target,
                                const EdgeFunc<T>& costs,
                                T bound) override;

  /**
   * A function returning a shortest path which is bounded
   * and which in addition satisfies the given filter.
   *
   * @tparam Filter  A filter given by a function mapping from Edge%s
   *                 to boolean values
   * @tparam bounded Whether or not to respect the given bound value.
   **/
  template<class Filter = AllEdgeFilter, bool bounded = false>
  SearchResult<T> shortest_path(Vertex source,
                                Vertex target,
                                const EdgeFunc<T>& costs,
                                const Filter& filter = Filter(),
                                T bound = inf);
};


template<class T>
template<class Filter, bool bounded>
SearchResult<T> Dijkstra<T>::shortest_path(Vertex source,
                                           Vertex target,
                                           const EdgeFunc<T>& costs,
                                           const Filter& filter,
                                           T bound)
{
  LabelHeap<Label<T>> heap(graph);
  int settled = 0, labeled = 0;
  bool found = false;

  heap.update(Label<T>(source, Edge(), 0));

  while(!heap.is_empty())
  {
    const Label<T>& current = heap.extract_min();

    ++settled;

    if(bounded)
    {
      if(current.get_cost() > bound)
      {
        break;
      }
    }

    if(current.get_vertex() == target)
    {
      found = true;
      break;
    }

    for(const Edge& edge : graph.get_outgoing(current.get_vertex()))
    {
      if(!filter(edge))
      {
        continue;
      }

      ++labeled;

      Label<T> next_label = Label<T>(edge.get_target(),
                                     edge, current.get_cost() + costs(edge));

      heap.update(next_label);
    }

  }

  if(found)
  {
    Path path;

    Label<T> current = heap.get_label(target);
    const num cost = current.get_cost();

    while((current.get_vertex() != source))
    {
      Edge edge = current.get_edge();
      path.prepend(edge);
      current = heap.get_label(edge.get_source());
    }

    return SearchResult<T>(settled, labeled, true, path, cost);
  }

  return SearchResult<T>::not_found(settled, labeled);
}


template<class T>
SearchResult<T> Dijkstra<T>::shortest_path(Vertex source,
                                           Vertex target,
                                           const EdgeFunc<T>& costs)
{
  return shortest_path<AllEdgeFilter, false>(source,
                                             target,
                                             costs,
                                             AllEdgeFilter(),
                                             inf);
}

template<class T>
SearchResult<T> Dijkstra<T>::shortest_path(Vertex source,
                                           Vertex target,
                                           const EdgeFunc<T>& costs,
                                           T bound)
{
  if(bound == inf)
  {
    return shortest_path<AllEdgeFilter, false>(source,
                                               target,
                                               costs,
                                               AllEdgeFilter(),
                                               bound);
  }
  else
  {
    return shortest_path<AllEdgeFilter, true>(source,
                                              target,
                                              costs,
                                              AllEdgeFilter(),
                                              bound);
  }
}


#endif /* ROUTER_HH */
