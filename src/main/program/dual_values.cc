#include "dual_values.hh"


DualValues::DualValues(const EdgeMap<double>& edge_values,
                       double path_cost,
                       double objective_value,
                       const RequestGraph& request_graph,
                       int max_num_paths)
  : edge_values(edge_values),
    path_cost(path_cost),
    objective_value(objective_value),
    request_graph(request_graph),
    max_num_paths(max_num_paths)
{
  assert(!cmp::pos(path_cost));

  assert(max_num_paths >= 0 || max_num_paths == -1);
}

DualValues::DualValues(const RequestGraph& request_graph,
                       int max_num_paths)
  : edge_values(request_graph, 0),
    path_cost(0),
    objective_value(0),
    request_graph(request_graph),
    max_num_paths(max_num_paths)
{
  assert(!cmp::pos(path_cost));

  assert(max_num_paths >= 0 || max_num_paths == -1);
}

double DualValues::operator()(const Edge& edge) const
{
  double cost = edge_values(edge);

  if(edge.get_source() == request_graph.get_origin())
  {
    return cost + path_cost;
  }

  return cost;
}

DualValues& DualValues::operator=(const DualValues& other)
{
  assert(&get_request_graph() == &other.get_request_graph());

  assert(get_max_num_paths() == other.get_max_num_paths());

  edge_values = other.edge_values;
  path_cost = other.path_cost;
  objective_value = other.objective_value;

  return *this;
}

DualValues::operator EdgeMap<double>() const
{
  const Graph& graph = request_graph;
  EdgeMap<double> values(graph ,0);

  for(const Edge& edge : graph.get_edges())
  {
    values(edge) = (*this)(edge);
  }

  return values;
}
