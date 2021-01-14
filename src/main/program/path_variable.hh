#ifndef PATH_VARIABLE_HH
#define PATH_VARIABLE_HH

#include "path/path.hh"
#include "scip_utils.hh"

#include "request_pattern.hh"

class PathVariable
{
private:
  SCIP_VAR* var;
  idx index;
  Path path;
  RequestPattern pattern;

public:
  PathVariable(SCIP_VAR* var,
               idx index,
               Path path,
               const RequestGraph& request_graph)
    : var(var),
      index(index),
      path(path),
      pattern(request_graph.get_instance(),
              request_graph.get_edge_types(),
              request_graph.get_edge_requests(),
              path)
  {
    assert(var);
  }

  SCIP_VAR* get_var() const
  {
    return var;
  }

  const RequestPattern& get_pattern() const
  {
    return pattern;
  }

  const Path& get_path() const
  {
    return path;
  }

  double get_cost() const
  {
    return SCIPvarGetObj(get_var());
  }

  idx get_index() const
  {
    return index;
  }
};


#endif /* PATH_VARIABLE_HH */
