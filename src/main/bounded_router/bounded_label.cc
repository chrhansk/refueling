#include "bounded_label.hh"

bool BoundedLabelComparator::operator()(const std::shared_ptr<BoundedLabel>& first,
                                        const std::shared_ptr<BoundedLabel>& second) const
{
  return (*second) < (*first);
}

BoundedLabel::BoundedLabel(double cost,
                           Units::Mass fuel,
                           std::shared_ptr<BoundedLabel> predecessor,
                           const Edge& edge)
  : cost(cost),
    fuel(fuel),
    predecessor(predecessor),
    edge(edge),
    vertex(edge.get_source())
{}


bool BoundedLabel::dominates(const BoundedLabel& other) const
{
  return get_cost() <= other.get_cost() &&
    get_fuel() <= other.get_fuel();
}

double BoundedLabel::get_cost() const
{
  return cost;
}

Units::Mass BoundedLabel::get_fuel() const
{
  return fuel;
}

std::shared_ptr<BoundedLabel> BoundedLabel::get_predecessor() const
{
  return predecessor;
}

const Edge& BoundedLabel::get_edge() const
{
  return edge;
}

bool BoundedLabel::operator<(const BoundedLabel& other) const
{
  if(get_cost() != other.get_cost())
  {
    return get_cost() < other.get_cost();
  }
  else
  {
    return get_fuel() < other.get_fuel();
  }
}

Vertex BoundedLabel::get_vertex() const
{
  return vertex;
}
