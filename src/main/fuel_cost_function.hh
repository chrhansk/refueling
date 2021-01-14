#ifndef FUEL_COST_FUNCTION_HH
#define FUEL_COST_FUNCTION_HH

#include "graph/graph.hh"
#include "graph/edge_map.hh"
#include "units.hh"

#include "op_type.hh"
#include "instance.hh"

#include "graph/graph.hh"
#include "path/path.hh"
#include "edge_fuel_difference.hh"

template<class R, class T, class Func> class ChainedFuelCostFunction;

template<class T>
class FuelCostFunction
{
public:
  virtual T operator()(const Edge& edge,
                       const Units::Mass& initial_fuel,
                       const Units::Mass& final_fuel) const = 0;

  T cost(const Path& path,
         const EdgeFuelDifference& fuel_difference,
         const Units::Mass& final_fuel = Units::Mass(0 * Units::SI::kilogram)) const;

  template<class R, class Func>
  ChainedFuelCostFunction<R, T, Func> then(const Func& func);

  virtual bool integral() const
  {
    return false;
  }

  virtual bool nonnegative() const
  {
    return true;
  }
};

template<class R, class T, class Func>
class ChainedFuelCostFunction : public FuelCostFunction<R>
{
private:
  const FuelCostFunction<T>& orig;
  Func func;
public:
  ChainedFuelCostFunction(const FuelCostFunction<T>& orig,
                          const Func& func)
    : orig(orig),
      func(func)
  {}

  virtual R operator()(const Edge& edge,
                       const Units::Mass& initial_fuel,
                       const Units::Mass& final_fuel) const override
  {
    return func(orig(edge, initial_fuel, final_fuel));
  }

};

template<class T>
template<class R, class Func>
ChainedFuelCostFunction<R, T, Func>
FuelCostFunction<T>::then(const Func& func)
{
  return ChainedFuelCostFunction<R, T, Func>(*this, func);
}

template<class T>
T FuelCostFunction<T>::cost(const Path& path,
                            const EdgeFuelDifference& fuel_difference,
                            const Units::Mass& final_fuel) const
{
  T current_cost = 0;

  auto it = path.get_edges().rbegin();
  auto end = path.get_edges().rend();

  Units::Mass current_fuel = final_fuel;

  for(; it != end; ++it)
  {
    const Edge& edge = *it;
    Units::Mass initial_fuel = fuel_difference(edge, current_fuel);

    current_cost += (*this)(edge, initial_fuel, current_fuel);

    current_fuel = initial_fuel;
  }

  return current_cost;
}

class DeliveredFuel : public FuelCostFunction<Units::Mass>
{
private:
  const EdgeMap<Request>& edge_requests;
  const EdgeMap<OpType>& edge_types;

public:
  DeliveredFuel(const EdgeMap<Request>& edge_requests,
                const EdgeMap<OpType>& edge_types);
public:
  Units::Mass operator()(const Edge& edge,
                         const Units::Mass& initial_fuel,
                         const Units::Mass& final_fuel) const override;
};

class BurnedFuel : public FuelCostFunction<Units::Mass>
{
private:
  const EdgeMap<Request>& edge_requests;
  const EdgeMap<OpType>& edge_types;
  DeliveredFuel delivered_fuel;

public:
  BurnedFuel(const EdgeMap<Request>& edge_requests,
             const EdgeMap<OpType>& edge_types);

  Units::Mass operator()(const Edge& edge,
                         const Units::Mass& initial_fuel,
                         const Units::Mass& final_fuel) const override;
};

class PathCount : public FuelCostFunction<double>
{
private:
  Vertex vertex;

public:
  PathCount(const Vertex& vertex);

  double operator()(const Edge& edge,
                    const Units::Mass& initial_fuel,
                    const Units::Mass& final_fuel) const override;

  virtual bool integral() const override
  {
    return true;
  }
};

class ZeroFunction : public FuelCostFunction<double>
{
  double operator()(const Edge& edge,
                    const Units::Mass& initial_fuel,
                    const Units::Mass& final_fuel) const override
  {
    return 0.;
  }

  virtual bool integral() const override
  {
    return true;
  }
};


class NegRequestCount : public FuelCostFunction<double>
{
private:
  const EdgeFunc<OpType>& edge_types;

public:
  NegRequestCount(const EdgeFunc<OpType>& edge_types);

  double operator()(const Edge& edge,
                    const Units::Mass& initial_fuel,
                    const Units::Mass& final_fuel) const override;

  virtual bool integral() const
  {
    return true;
  }

  virtual bool nonnegative() const
  {
    return false;
  }
};



#endif /* FUEL_COST_FUNCTION_HH */
