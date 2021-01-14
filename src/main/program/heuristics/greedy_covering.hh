#ifndef GREEDY_COVERING_HH
#define GREEDY_COVERING_HH

#include "request_graph.hh"

#include "program/request_pattern.hh"

#include "path/path.hh"

class GreedyCovering
{
public:
  class Result
  {
  private:
    std::vector<Path> paths;
    double cost;

    Result()
    {}

  public:
    Result(const std::vector<Path>& paths,
           double cost)
      : paths(paths),
        cost(cost)
    {}

    const std::vector<Path>& get_paths() const
    {
      return paths;
    }

    static Result empty()
    {
      return Result();
    }

    operator bool() const
    {
      return !paths.empty();
    }

    double get_cost() const
    {
      return cost;
    }
  };

private:
  const RequestGraph& request_graph;

  struct RequestPath
  {
    Path path;
    double cost;
    RequestPattern pattern;
  };

  std::vector<RequestPath> request_paths;
public:

  GreedyCovering(const RequestGraph& request_graph)
    : request_graph(request_graph)
  {}

  void add_path(const Path& path,
                double cost);

  void add_path(const Path& path,
                double cost,
                const RequestPattern& pattern);

  Result compute(idx max_num_paths);
};


#endif /* GREEDY_COVERING_HH */
