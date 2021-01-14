#ifndef LABEL_HEAP_HH
#define LABEL_HEAP_HH

#include <boost/heap/d_ary_heap.hpp>

#include "graph/graph.hh"
#include "graph/vertex_map.hh"

template <class T>
using SimpleHeap = boost::heap::d_ary_heap<T,
                                           boost::heap::mutable_<true>,
                                           boost::heap::compare<std::greater<T>>,
                                           boost::heap::arity<2>>;

template <class Label>
class LabelHeap
{
public:
  typedef SimpleHeap<Label> Heap;
private:
  typedef typename Heap::handle_type Handle;

  const Graph& graph;
  VertexMap<Label> labels;
  VertexMap<Handle> handles;
  Heap heap;
public:
  LabelHeap(const Graph& graph);

  const Label& get_label(Vertex vertex) const;
  Label& get_label(Vertex vertex);
  void update(Label label);
  const Label& extract_min();

  const Label& peek();

  bool finished() const;

  bool is_empty() const;
};

template <class Label>
LabelHeap<Label>::LabelHeap(const Graph& graph)
  : graph(graph),
    labels(graph, Label()),
    handles(graph, Handle())
{
}

template <class Label>
const Label& LabelHeap<Label>::get_label(Vertex vertex) const
{
  return labels(vertex);
}

template <class Label>
Label& LabelHeap<Label>::get_label(Vertex vertex)
{
  return labels(vertex);
}

template <class Label>
void LabelHeap<Label>::update(Label label)
{
  Label& current = get_label(label.get_vertex());

  switch(current.get_state())
  {
  case State::UNKNOWN:
    current = label;
    current.set_state(State::LABELED);
    handles(current.get_vertex()) = heap.push(current);
    break;
  case State::SETTLED:
    return;
  case State::LABELED:
    if(current > label)
    {
      Handle handle = handles(current.get_vertex());
      heap.update(handle, label);
      current = label;
    }
    return;
  }

}

template <class Label>
const Label& LabelHeap<Label>::extract_min()
{
  assert(!is_empty());
  const Label minLabel = heap.top();
  heap.pop();
  Label& label = get_label(minLabel.get_vertex());

  label.set_state(State::SETTLED);

  return label;
}

template <class Label>
const Label& LabelHeap<Label>::peek()
{
  assert(!is_empty());
  const Label& minLabel = heap.top();
  Label& label = get_label(minLabel.get_vertex());
  return label;
}

template <class Label>
bool LabelHeap<Label>::finished() const
{
  for(const Vertex& vertex : graph.get_vertices())
  {
    if(get_label(vertex).get_state() == State::LABELED)
    {
      return false;
    }
  }

  return true;
}

template <class Label>
bool LabelHeap<Label>::is_empty() const
{
  return heap.empty();
}

#endif /* LABEL_HEAP_HH */
