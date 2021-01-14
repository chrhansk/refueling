#include "point.hh"

#include "util.hh"

Units::Length
Point::distance(const Point& other) const
{
  Units::Length radius(6372800 * Units::SI::meter);

  double lat_diff = deg_to_rad(other.get_lat() - get_lat());
  double lon_diff = deg_to_rad(other.get_lon() - get_lon());
  double lat1 = deg_to_rad(get_lat());
  double lat2 = deg_to_rad(other.get_lat());

  double a = sin(lat_diff / 2) * sin(lat_diff / 2) +
    sin(lon_diff / 2) * sin(lon_diff / 2) * cos(lat1) * cos(lat2);

  radius *= (2 * asin(sqrt(a)));

  return radius;
}

bool Point::operator==(const Point& other) const
{
  return get_lat() == other.get_lat() &&
    get_lon() == other.get_lon();
}

bool Point::operator!=(const Point& other) const
{
  return !(*this == other);
}
