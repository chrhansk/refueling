#ifndef POINT_HH
#define POINT_HH

#include "units.hh"

class Point
{
  double lat, lon;

public:
  Point(double lat, double lon)
    : lat(lat),
      lon(lon)
  {}

  Point()
    : lat(0),
      lon(0)
  {}

  void set_lat(double latitude)
  {
    lat = latitude;
  }

  void set_lon(double longitude)
  {
    lon = longitude;
  }

  double get_lat() const
  {
    return lat;
  }

  double get_lon() const
  {
    return lon;
  }

  // haversine distance to another point
  Units::Length distance(const Point& other) const;

  bool operator==(const Point& other) const;

  bool operator!=(const Point& other) const;
};


#endif /* POINT_HH */
