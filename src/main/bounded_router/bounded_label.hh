#ifndef BOUNDED_LABEL_HH
#define BOUNDED_LABEL_HH

#include <boost/heap/d_ary_heap.hpp>
#include <memory>

#include "units.hh"

#include "graph/graph.hh"

class BoundedLabel;

struct BoundedLabelComparator
{
  bool operator()(const std::shared_ptr<BoundedLabel>& first,
                  const std::shared_ptr<BoundedLabel>& second) const;
};

class BoundedLabel
{
private:
  double cost;
  Units::Mass fuel;

  std::shared_ptr<BoundedLabel> predecessor;
  Edge edge;
  Vertex vertex;

public:
  BoundedLabel(const Vertex& vertex)
    : cost(0),
      fuel(0 * Units::SI::kilogram),
      vertex(vertex)
  {}

  BoundedLabel(double cost,
               Units::Mass fuel,
               std::shared_ptr<BoundedLabel> predecessor,
               const Edge& edge);

  bool dominates(const BoundedLabel& other) const;

  double get_cost() const;

  Units::Mass get_fuel() const;

  std::shared_ptr<BoundedLabel> get_predecessor() const;

  const Edge& get_edge() const;

  bool operator<(const BoundedLabel& other) const;

  Vertex get_vertex() const;
};

typedef std::shared_ptr<BoundedLabel> BoundedLabelPtr;

#endif /* BOUNDED_LABEL_HH */
