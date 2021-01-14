#include "gtest/gtest.h"

#include "log.hh"

#include "fuel_difference.hh"

TEST(FuelDifference, Simple)
{
  log_init();

  Point origin(51.2495, -10.3929);
  Point destination(52.6994, -8.92193);
  Parameters parameters;

  Units::Length distance = origin.distance(destination);

  Units::Time flight_duration = distance / parameters.get_flight_speed();

  Units::Mass final_fuel(0*Units::SI::kilogram);

  FuelDifference fuel_difference(parameters);

  Units::Mass initial_fuel = fuel_difference.descent_initial_fuel(flight_duration,
                                                                  final_fuel);

  Units::Mass expected_initial_fuel(43.6105 * Units::SI::kilogram);

  EXPECT_TRUE(cmp::eq(initial_fuel / Units::SI::kilogram, expected_initial_fuel / Units::SI::kilogram, 1e-3));
}
