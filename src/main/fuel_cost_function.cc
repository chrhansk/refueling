#include "fuel_cost_function.hh"

DeliveredFuel::DeliveredFuel(const EdgeMap<Request>& edge_requests,
                             const EdgeMap<OpType>& edge_types)
  : edge_requests(edge_requests),
    edge_types(edge_types)
{

}


Units::Mass DeliveredFuel::operator()(const Edge& edge,
                                      const Units::Mass& initial_fuel,
                                      const Units::Mass& final_fuel) const
{
  OpType edge_type = edge_types(edge);

  if(edge_type == OpType::REFUELING)
  {
    return edge_requests(edge).get_amount();
  }
  else
  {
    return Units::Mass(0 * Units::SI::kilogram);
  }
}

BurnedFuel::BurnedFuel(const EdgeMap<Request>& edge_requests,
                       const EdgeMap<OpType>& edge_types)
  : edge_requests(edge_requests),
    edge_types(edge_types),
    delivered_fuel(edge_requests, edge_types)
{}

Units::Mass BurnedFuel::operator()(const Edge& edge,
                                   const Units::Mass& initial_fuel,
                                   const Units::Mass& final_fuel) const
{
  OpType edge_type = edge_types(edge);

  switch (edge_type)
  {
  case OpType::FLIGHT:
  case OpType::CLIMB:
  case OpType::DESCENT:
    return initial_fuel - final_fuel;
  case OpType::REFUELING:
    return (initial_fuel - final_fuel) - delivered_fuel(edge, initial_fuel, final_fuel);
  default:
    return Units::Mass(0 * Units::SI::kilogram);
  }

}

PathCount::PathCount(const Vertex& vertex)
  : vertex(vertex)
{

}

double PathCount::operator()(const Edge& edge,
                             const Units::Mass& initial_fuel,
                             const Units::Mass& final_fuel) const
{
  return int(edge.get_source() == vertex);
}

NegRequestCount::NegRequestCount(const EdgeFunc<OpType>& edge_types)
  : edge_types(edge_types)
{

}

double NegRequestCount::operator()(const Edge& edge,
                                   const Units::Mass& initial_fuel,
                                   const Units::Mass& final_fuel) const
{
  return edge_types(edge) == OpType::REFUELING ? -1 : 0;
}
