#ifndef ARTIFICIAL_VARIABLE_HH
#define ARTIFICIAL_VARIABLE_HH

#include "program/scheduling_program.hh"

class ArtificialVariable
{
private:
  SCIP_VAR* var;
  const Request& request;

public:
  ArtificialVariable(SCIP_VAR* var,
                     const Request& request)
    : var(var),
      request(request)
  {}

  SCIP_VAR* get_var() const
  {
    return var;
  }

  const Request& get_request() const
  {
    return request;
  }
};


#endif /* ARTIFICIAL_VARIABLE_HH */
