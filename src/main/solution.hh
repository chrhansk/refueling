#ifndef SOLUTION_HH
#define SOLUTION_HH

#include "op_type.hh"

#include "instance.hh"
#include "parameters.hh"

class Solution
{
public:

  class Step
  {
  private:
    OpType type;
    TimeSpan timespan;
    Point origin, destination;
    Units::Mass initial_fuel, final_fuel, burned_fuel;
    std::shared_ptr<Request> request;

    Step(OpType type,
         TimeSpan timespan,
         Point origin,
         Point destination,
         Units::Mass initial_fuel,
         Units::Mass final_fuel,
         Units::Mass burned_fuel,
         std::shared_ptr<Request> request)
      : type(type),
        timespan(timespan),
        origin(origin),
        destination(destination),
        initial_fuel(initial_fuel),
        final_fuel(final_fuel),
        burned_fuel(burned_fuel),
        request(request)
    {}

  public:

    static Step refueling(const TimeSpan& timespan,
                          const Point& origin,
                          const Point& destination,
                          Units::Mass initial_fuel,
                          Units::Mass final_fuel,
                          Units::Mass burned_fuel,
                          const Request& request)
    {
      return Step(OpType::REFUELING,
                  timespan,
                  origin,
                  destination,
                  initial_fuel,
                  final_fuel,
                  burned_fuel,
                  std::make_shared<Request>(request));
    }

    static Step base_waiting(const TimeSpan& timespan,
                             const Point& origin)
    {
      Units::Mass zero(0*Units::SI::kilogram);

      return Step(OpType::BASE_WAITING,
                  timespan,
                  origin,
                  origin,
                  zero,
                  zero,
                  zero,
                  std::shared_ptr<Request>());
    }

    static Step create(OpType type,
                       const TimeSpan& timespan,
                       const Point& origin,
                       const Point& destination,
                       Units::Mass initial_fuel,
                       Units::Mass final_fuel,
                       Units::Mass burned_fuel)
    {
      assert(type != OpType::REFUELING);

      return Step(type,
                  timespan,
                  origin,
                  destination,
                  initial_fuel,
                  final_fuel,
                  burned_fuel,
                  std::shared_ptr<Request>());
    }

    OpType get_type() const
    {
      return type;
    }

    Units::Mass get_initial_fuel() const
    {
      return initial_fuel;
    }

    Units::Mass get_final_fuel() const
    {
      return final_fuel;
    }

    Units::Mass get_burned_fuel() const
    {
      return burned_fuel;
    }

    const TimeSpan& get_timespan() const
    {
      return timespan;
    }

    const Point& get_origin() const
    {
      return origin;
    }

    const Point& get_destination() const
    {
      return destination;
    }

    std::shared_ptr<Request> get_request() const
    {
      return request;
    }

    bool is_valid(const Parameters& parameters) const;
  };

  class Path
  {
  private:
    std::vector<Step> steps;

  public:

    void append(const Path& path)
    {
      for(const auto& step : path.get_steps())
      {
        append(step);
      }
    }

    void append(const Step& step)
    {
      if(!steps.empty())
      {
        const Step& last_step = steps.back();

        assert(last_step.get_destination() == step.get_origin());
        assert(last_step.get_timespan().get_end() == step.get_timespan().get_begin());
      }

      steps.push_back(step);
    }

    const std::vector<Step>& get_steps() const
    {
      return steps;
    }

    TimeSpan get_timespan() const
    {
      assert(!steps.empty());

      return TimeSpan(steps.front().get_timespan().get_begin(),
                      steps.back().get_timespan().get_end());
    }

    const Point& get_origin() const
    {
      assert(!steps.empty());

      return steps.front().get_origin();
    }

    const Point& get_destination() const
    {
      assert(!steps.empty());

      return steps.back().get_destination();
    }

    Units::Mass get_initial_fuel() const
    {
      return steps.front().get_initial_fuel();
    }

    Units::Mass get_final_fuel() const
    {
      return steps.back().get_final_fuel();
    }


    bool is_valid(const Instance& instance,
                  const Parameters& parameters) const;

  };

private:
  std::vector<Path> paths;
  double gap;

  bool serves_all_requests(const Instance& instance) const;
  bool has_valid_paths(const Instance& instance,
                       const Parameters& parameters) const;

public:

  Solution(double gap = 0.)
    : gap(gap)
  {}

  Solution(const Solution& other) = default;

  Solution(Solution&& other)
    : paths(std::move(other.paths)),
      gap(other.gap)
  {}

  Solution& operator=(const Solution& other) = default;

  Solution& operator=(Solution&& other)
  {
    paths = std::move(other.paths);
    gap = other.gap;
    return *this;
  }

  const std::vector<Path>& get_paths() const
  {
    return paths;
  }

  std::vector<Path>& get_paths()
  {
    return paths;
  }

  double get_gap() const
  {
    return gap;
  }

  Units::Mass total_burned_fuel() const;

  bool is_valid(const Instance& instance,
                const Parameters& parameters) const;

};

std::ostream& operator<<(std::ostream& out, const Solution& solution);

#endif /* SOLUTION_HH */
