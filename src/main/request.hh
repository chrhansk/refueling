#ifndef REQUEST_HH
#define REQUEST_HH

#include "util.hh"
#include "time.hh"

class Request
{
private:
  idx index;
  idx flight;
  DateTime time;
  Point origin;
  double direction;
  Units::Mass amount;
  Point destination;

public:
  Request(idx index,
          idx flight,
          DateTime time,
          const Point& origin,
          double direction,
          const Units::Mass& amount,
          const Point& destination);

  Request();

  bool operator==(const Request& other) const;
  bool operator!=(const Request& other) const;

  idx get_index() const;

  void set_index(idx ind);

  idx get_flight() const;

  void set_flight(idx ind);

  double _get_time() const;

  void _set_time(double ind);

  const Point& get_origin() const;

  void set_origin(const Point& ind);

  double get_direction() const;

  void set_direction(double ind);

  Units::Mass get_amount() const;

  void set_amount(const Units::Mass& value);

  double _get_amount() const;

  void _set_amount(double val);

  const Point& get_destination() const;

  void set_destination(const Point& ind);

  const DateTime& get_time() const;

  bool operator<(const Request& other) const
  {
    return get_index() < other.get_index();
  }
};


#endif /* REQUEST_HH */
