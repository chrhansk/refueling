#include "request.hh"

Request::Request(idx index,
                 idx flight,
                 DateTime time,
                 const Point& origin,
                 double direction,
                 const Units::Mass& amount,
                 const Point& destination)
  : index(index),
    flight(flight),
    time(time),
    origin(origin),
    direction(direction),
    amount(amount),
    destination(destination)
{
}

Request::Request()
{}

bool Request::operator==(const Request& other) const
{
  return get_index() == other.get_index();
}

bool Request::operator!=(const Request& other) const
{
  return !(*this == other);
}

idx Request::get_index() const
{
  return index;
}

void Request::set_index(idx ind)
{
  index = ind;
}

idx Request::get_flight() const
{
  return flight;
}

void Request::set_flight(idx ind)
{
  flight = ind;
}

double Request::_get_time() const
{
  return fractional_seconds(time - initial_time) / 3600;
}

void Request::_set_time(double ind)
{
  time = initial_time + fractional_seconds((ind * 3600));
}

const Point& Request::get_origin() const
{
  return origin;
}

void Request::set_origin(const Point& ind)
{
  origin = ind;
}

double Request::get_direction() const
{
  return direction;
}

void Request::set_direction(double ind)
{
  direction = ind;
}

Units::Mass Request::get_amount() const
{
  return amount;
}

void Request::set_amount(const Units::Mass& value)
{
  amount = value;
}

double Request::_get_amount() const
{
  return (amount / Units::SI::kilogram);
}

void Request::_set_amount(double val)
{
  amount = Units::Mass(val * Units::Imperial::Pound::unit_type());
}

const Point& Request::get_destination() const
{
  return destination;
}

void Request::set_destination(const Point& ind)
{
  destination = ind;
}

const DateTime& Request::get_time() const
{
  return time;
}
