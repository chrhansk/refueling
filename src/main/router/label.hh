#ifndef LABEL_HH
#define LABEL_HH

#include "graph/graph.hh"

enum class State
{
  UNKNOWN, LABELED, SETTLED
};

template<class T = num>
class AbstractLabel
{
private:
  Vertex vertex;
  T cost;
  State state;

public:
  AbstractLabel()
    : cost(inf),
      state(State::UNKNOWN)
  {}
  AbstractLabel(Vertex vertex, T cost)
    : vertex(vertex),
      cost(cost),
      state(State::LABELED)
  {}

  T get_cost() const
  {
    return cost;
  }

  bool operator<(const AbstractLabel& other) const
  {
    return cost < other.cost;
  }

  bool operator>(const AbstractLabel& other) const
  {
    return cost > other.cost;
  }

  Vertex get_vertex() const
  {
    return vertex;
  }

  State get_state() const
  {
    return state;
  }

  void set_state(State state)
  {
    this->state = state;
  }

};

template<class T = num>
class SimpleLabel : public AbstractLabel<T>
{
public:
  SimpleLabel(Vertex vertex, T cost)
    : AbstractLabel<T>(vertex, cost) {}
  SimpleLabel() {}
};

template<class T = num>
class Label : public AbstractLabel<T>
{
private:
  Edge edge;

public:
  Label(Vertex vertex, const Edge& edge, T cost)
    : AbstractLabel<T>(vertex, cost),
      edge(edge)
  {}

  Label() {}

  Edge get_edge() const
  {
    return edge;
  }
};

#endif /* LABEL_HH */
