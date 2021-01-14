#ifndef FIXED_EDGE_HH
#define FIXED_EDGE_HH

#include <scip/scip.h>
#include <scip/scipdefplugins.h>

#include "graph/edge.hh"

enum class FixedType
{
  Lower,
  Upper
};

class FixedEdge
{
private:
  SCIP_CONS* cons;
  Edge edge;
  FixedType type;

public:
  FixedEdge(SCIP_CONS* cons,
            const Edge& edge,
            FixedType type)
    : cons(cons),
      edge(edge),
      type(type)
  {}

  const Edge& get_edge() const
  {
    return edge;
  }

  SCIP_CONS* get_constraint() const
  {
    return cons;
  }

  void set_constraint(SCIP_CONS* constraint)
  {
    cons = constraint;
  }

  FixedType get_type() const
  {
    return type;
  }

};


#endif /* FIXED_EDGE_HH */
