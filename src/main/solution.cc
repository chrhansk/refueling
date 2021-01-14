#include "solution.hh"

#include "program/request_pattern.hh"

#include "fuel_difference.hh"

static void print_step(std::ostream& out,
                       const Solution::Step& step,
                       int index)
{
  Point origin = step.get_origin();
  Point destination = step.get_destination();

  out << "Tanker " << index << " moves from"
      << " lat " << origin.get_lat()
      << " lon " << origin.get_lon()
      << " to"
      << " endlat " << destination.get_lat()
      << " endlon " << destination.get_lon();

  TimeSpan timespan = step.get_timespan();

  out << ", leaves at " << ((timespan.get_begin() - initial_time).total_seconds()) / 3600. << " [h]";
  out << ", arrives at " << ((timespan.get_end() - initial_time).total_seconds()) / 3600. << " [h]";

  out << ", fuel at origin: "
      << step.get_initial_fuel()
      << ", fuel at destination: "
      << step.get_final_fuel()
      << ", ";

  out << "burned fuel: " << step.get_burned_fuel() << ", ";

  if(step.get_type() == OpType::REFUELING)
  {
    auto request = step.get_request();

    out << "serving " << request->get_amount() << " to cruiser " << request->get_flight() << ", ";
  }

  Units::Length distance = origin.distance(destination);

  out << "distance: " << distance << ", ";

  out << "travel time: " << duration_time(timespan.duration()) << std::endl;

}

static void print_path(std::ostream& out,
                       const Solution::Path& path,
                       int index)
{
  for(const auto& step : path.get_steps())
  {
    print_step(out, step, index);
  }
}

Units::Mass Solution::total_burned_fuel() const
{
  Units::Mass burned_fuel(0*Units::SI::kilogram);

  for(const auto& path : get_paths())
  {
    for(const auto& step : path.get_steps())
    {
      burned_fuel += step.get_burned_fuel();
    }
  }

  return burned_fuel;
}

bool Solution::serves_all_requests(const Instance& instance) const
{
  RequestPattern pattern(instance);

  for(const auto& path : get_paths())
  {
    for(const auto& step : path.get_steps())
    {
      auto request = step.get_request();

      if(request)
      {
        if(pattern.contains(*request))
        {
          return false;
        }

        pattern.insert(*request);
      }
    }
  }

  if(pattern != RequestPattern::all(instance))
  {
    return false;
  }

  return true;
}

bool Solution::has_valid_paths(const Instance& instance,
                               const Parameters& parameters) const
{
  for(const auto& path : get_paths())
  {
    if(!path.is_valid(instance, parameters))
    {
      return false;
    }
  }

  return true;
}

bool Solution::is_valid(const Instance& instance,
                        const Parameters& parameters) const
{
  return serves_all_requests(instance) && has_valid_paths(instance, parameters);
}

bool Solution::Step::is_valid(const Parameters& parameters) const
{
  Units::Mass actual_initial_fuel;

  Units::Time duration = duration_time(get_timespan().duration());

  const Units::Mass refueling_amount = parameters.get_refueling_amount();

  FuelDifference fuel_difference(parameters);

  const Units::Mass zero(0*Units::SI::kilogram);

  auto request = get_request();

  switch(get_type())
  {
  case OpType::BASE_REFUELING:
    actual_initial_fuel = zero;
    break;
  case OpType::BASE_WAITING:
    actual_initial_fuel = get_final_fuel();
    break;
  case OpType::FLIGHT:
    actual_initial_fuel = fuel_difference.flight_initial_fuel(duration, get_final_fuel());
    break;
  case OpType::DESCENT:
    actual_initial_fuel = fuel_difference.descent_initial_fuel(duration, get_final_fuel());
    break;
  case OpType::REFUELING:
    assert(request);
    actual_initial_fuel = fuel_difference.refueling_initial_fuel(request->get_amount(), get_final_fuel());
    break;
  case OpType::CLIMB:
    actual_initial_fuel = fuel_difference.climb_initial_fuel(duration, get_final_fuel());
    break;
  }

  auto flight_speed = parameters.get_flight_speed();

  if(get_type() == OpType::BASE_REFUELING)
  {
    if(duration_time(get_timespan().duration()) != parameters.get_base_duration())
    {
      return false;
    }
  }

  if(get_type() == OpType::REFUELING)
  {
    if(duration_time(get_timespan().duration()) != parameters.get_refueling_duration())
    {
      return false;
    }
  }

  if(get_type() == OpType::CLIMB)
  {
    auto time = duration_time(get_timespan().duration());

    auto takeoff_distance = parameters.get_takeoff().get_distance();
    auto takeoff_time = takeoff_distance / flight_speed;

    double time_value = time / ( Units::SI::second);
    double takeoff_time_value = takeoff_time / ( Units::SI::second);

    if(cmp::gt(takeoff_time_value, time_value, 1e-5))
    {
      return false;
    }
  }

  {
    auto distance = get_origin().distance(get_destination());
    auto time = duration_time(get_timespan().duration());

    assert(time > 0 * Units::SI::second);

    auto actual_speed = distance / time;

    double actual_speed_value = actual_speed / (Units::SI::meter / Units::SI::second);
    double max_speed_value = flight_speed / (Units::SI::meter / Units::SI::second);

    if(cmp::gt(actual_speed_value, max_speed_value, 1e-5))
    {
      return false;
    }
  }

  if(get_initial_fuel() != actual_initial_fuel)
  {
    return false;
  }

  if(get_final_fuel() > refueling_amount)
  {
    return false;
  }

  if(get_initial_fuel() > refueling_amount)
  {
    return false;
  }

  return true;
}

bool Solution::Path::is_valid(const Instance& instance,
                              const Parameters& parameters) const
{
  if(steps.empty())
  {
    return false;
  }

  if(get_origin() != instance.get_origin())
  {
    return false;
  }

  if(get_destination() != instance.get_origin())
  {
    return false;
  }

  DateTime time = get_timespan().get_begin();
  Point point = get_origin();

  Units::Mass fuel = get_initial_fuel();

  for(const auto& step : steps)
  {
    if(point != step.get_origin())
    {
      return false;
    }

    if(time != step.get_timespan().get_begin())
    {
      return false;
    }

    if(step.get_initial_fuel() != fuel)
    {
      return false;
    }

    if(!step.is_valid(parameters))
    {
      return false;
    }

    point = step.get_destination();
    time = step.get_timespan().get_end();
    fuel = step.get_final_fuel();
  }

  if(point != get_destination())
  {
    return false;
  }

  if(time != get_timespan().get_end())
  {
    return false;
  }

  if(fuel != get_final_fuel())
  {
    return false;
  }

  return true;
}

std::ostream& operator<<(std::ostream& out, const Solution& solution)
{
  int index = 0;

  for(const auto& path : solution.get_paths())
  {
    print_path(out, path, index++);
  }

  out << "Number of tankers: " << solution.get_paths().size() << std::endl;
  out << "Total burned fuel: " << solution.total_burned_fuel() << std::endl;

  if(!cmp::zero(solution.get_gap()))
  {
    out << "Remaining gap: " << solution.get_gap() << std::endl;
  }

  return out;
}
