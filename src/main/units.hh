#ifndef UNITS_HH
#define UNITS_HH

#include <boost/units/cmath.hpp>
#include <boost/units/io.hpp>
#include <boost/units/systems/si.hpp>
#include <boost/units/base_units/imperial/pound.hpp>
#include <boost/units/base_units/metric/hour.hpp>
#include <boost/units/base_units/metric/minute.hpp>
#include <boost/units/systems/si/io.hpp>
#include <boost/units/systems/si/prefixes.hpp>

namespace Units
{
  using namespace boost::units;

  namespace SI
  {
    using namespace boost::units::si;
  }

  namespace Imperial
  {
    using namespace boost::units::imperial;

    typedef typename Units::Imperial::pound_base_unit Pound;
  }

  namespace Metric
  {
    using namespace boost::units::metric;

    typedef typename Units::Metric::hour_base_unit Hour;
    typedef typename Units::Metric::minute_base_unit Minute;
  }

  namespace Nautical
  {
    struct length_base_unit :
      boost::units::base_unit<length_base_unit, length_dimension, 1>
    {
      static std::string name()       { return "nautical mile"; }
      static std::string symbol()     { return "nmi"; }
    };

    typedef Units::make_system<Units::Nautical::length_base_unit>::type system;

    typedef unit<length_dimension, system> length;

    static const length mile,miles;
  }

  template <class U> using Quantity = quantity<U>;

  typedef Quantity<SI::length> Length;
  typedef Quantity<Nautical::length> NauticalLength;
  typedef Quantity<SI::velocity> Velocity;
  typedef Quantity<SI::time> Time;
  typedef Quantity<SI::mass> Mass;

  typedef Units::derived_dimension<Units::mass_base_dimension, 1,
                                   Units::time_base_dimension, -1>::type MassFlowDimension;

  typedef boost::units::unit<MassFlowDimension, Units::SI::system> MassFlowUnit;

  //typedef Quantity<MassFlowDimension> MassFlowRate;

  typedef boost::units::quantity<MassFlowUnit, double> MassFlowRate;

  BOOST_UNITS_STATIC_CONSTANT(kilogram_per_seconds, MassFlowRate);
}

BOOST_UNITS_DEFINE_CONVERSION_FACTOR(Units::Nautical::length_base_unit,
                                     Units::SI::meter_base_unit,
                                     double, 1.852e3);

/// unit typedefs
/*
typedef unit<length_dimension, system> length;

static const length mile,miles;

// helper for conversions between nautical length and si length
BOOST_UNITS_DEFINE_CONVERSION_FACTOR(Units::Nautical::length_base_unit,
                                     Units::SI::meter_base_unit,
                                     double, 1.852e3);
*/
#endif /* UNITS_HH */
