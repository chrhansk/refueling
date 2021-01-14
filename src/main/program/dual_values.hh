#ifndef DUAL_VALUES_HH
#define DUAL_VALUES_HH

#include "graph/graph.hh"
#include "graph/edge_map.hh"
#include "graph/vertex_map.hh"

#include "request_graph.hh"

template<class T>
class RequestEdgeMap : public EdgeFunc<T>
{
private:
  RequestMap<T> values;
  T value;
  const RequestGraph* request_graph;

public:
  RequestEdgeMap(const RequestGraph& request_graph,
                 const RequestMap<T>& values,
                 T value = T())
    : values(values),
      value(value),
      request_graph(&request_graph)
  {}

  RequestEdgeMap(const RequestGraph& request_graph,
                 T value = T())
    : values(request_graph.get_instance().request_map(value)),
      value(value),
      request_graph(&request_graph)
  {}

  T operator()(const Edge& edge) const
  {
    OpType edge_type = request_graph->get_edge_types()(edge);

    if(edge_type == OpType::REFUELING)
    {
      const Request& request = request_graph->get_edge_requests()(edge);
      return values(request);
    }
    return value;
  }

  void set_value(const Request& request, const T& value)
  {
    values.set_value(request, value);
  }

  void set_value(const Edge& edge, const T& value)
  {
    const Request& request = request_graph->get_edge_requests()(edge);

    values.set_value(request, value);
  }
};

class DualValues : public EdgeFunc<double>
{
private:
  EdgeMap<double> edge_values;
  double path_cost;
  double objective_value;

  const RequestGraph& request_graph;
  int max_num_paths;

public:
  DualValues(const EdgeMap<double>& edge_values,
             double path_cost,
             double objective_value,
             const RequestGraph& request_graph,
             int max_num_paths);

  DualValues(const RequestGraph& request_graph,
             int max_num_paths);

  double operator()(const Edge& edge) const override;

  DualValues& operator=(const DualValues& other);

  const RequestGraph& get_request_graph() const
  {
    return request_graph;
  }

  double& get_objective_value()
  {
    return objective_value;
  }

  const double& get_objective_value() const
  {
    return objective_value;
  }

  int& get_max_num_paths()
  {
    return max_num_paths;
  }

  const int& get_max_num_paths() const
  {
    return max_num_paths;
  }

  operator EdgeMap<double>() const;

  EdgeMap<double>& get_edge_values()
  {
    return edge_values;
  }

  double& get_path_cost()
  {
    return path_cost;
  }

  const EdgeFunc<double>& get_edge_values() const
  {
    return edge_values;
  }

  double get_path_cost() const
  {
    return path_cost;
  }

};


#endif /* DUAL_VALUES_HH */
