#ifndef FUNDAMENTAL_PATH_DUAL_VALUES_HH
#define FUNDAMENTAL_PATH_DUAL_VALUES_HH

class FundamentalPathDualValues
{
private:
  EdgeMap<double> edge_values;

public:
  FundamentalPathDualValues(const FundamentalPathGraph& fundamental_path_graph)
    : edge_values(fundamental_path_graph, 0.)
  {}

  EdgeMap<double>& get_edge_values()
  {
    return edge_values;
  }

  const EdgeMap<double>& get_edge_values() const
  {
    return edge_values;
  }

  operator const EdgeFunc<double>&() const
  {
    return edge_values;
  }
};


#endif /* FUNDAMENTAL_PATH_DUAL_VALUES_HH */
