#include "instance_reader.hh"

#include "log.hh"

#include <boost/spirit/include/qi.hpp>

#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>

#include <boost/fusion/adapted/adt/adapt_adt.hpp>
#include <boost/fusion/include/adapt_adt.hpp>

#include "instance.hh"
#include "point.hh"
#include "request.hh"

BOOST_FUSION_ADAPT_ADT(
  Point,
  (obj.get_lat(), obj.set_lat(val))
  (obj.get_lon(), obj.set_lon(val)))

BOOST_FUSION_ADAPT_ADT(
  Request,
  (obj.get_index(),obj.set_index(val))
  (obj.get_flight(),obj.set_flight(val))
  (obj._get_time(),obj._set_time(val))
  (obj.get_origin(),obj.set_origin(val))
  (obj.get_direction(),obj.set_direction(val))
  (obj._get_amount(),obj._set_amount(val))
  (obj.get_destination(),obj.set_destination(val)))

BOOST_FUSION_ADAPT_ADT(
  Instance,
  (obj.get_origin(),obj.set_origin(val))
  (obj.get_requests(),obj.set_requests(val)))

Instance InstanceReader::read(std::istream& input)
{
  namespace spirit = boost::spirit;
  namespace qi = spirit::qi;
  namespace ascii = spirit::ascii;
  namespace phoenix = boost::phoenix;

  input.unsetf(std::ios::skipws);

  typedef spirit::istream_iterator iterator_type;

  iterator_type first(input);
  iterator_type last;

  typedef ascii::blank_type skipper_type;

  qi::rule<iterator_type, Point(), skipper_type> lat_lon_def =
    ("Lat" > qi::double_ > "Lon" > qi::double_);

  qi::rule<iterator_type, Point(), skipper_type> end_lat_lon_def =
    ("endlat" > qi::double_ > "endlon" > qi::double_);

  qi::rule<iterator_type, Point(), skipper_type> base_def = "Tankerbaseposition" > lat_lon_def > qi::eol;

  qi::rule<iterator_type, Request(), skipper_type> request_def =
    "Request" > qi::uint_ > "," >
    "flight" > qi::uint_ >
    "at" > qi::double_ > "[h]" >
    lat_lon_def > "," >
    "flying track" > qi::double_ >
    "for" > qi::double_ > "[lbs]" >
    end_lat_lon_def >
    qi::eol;

  qi::rule<iterator_type, std::vector<Request>(), skipper_type> requests_def =
    *(request_def[phoenix::push_back(phoenix::ref(qi::_val), qi::_1)]);

  qi::rule<iterator_type, Instance(), skipper_type> instance_def = base_def >> requests_def;

  Instance instance;

  try {

    bool success = qi::phrase_parse(first,
                                    last,
                                    instance_def,
                                    ascii::blank,
                                    instance);

    if(!success || first != last)
    {
      throw std::invalid_argument("Failed to parse file");
    }

  }
  catch(qi::expectation_failure<iterator_type> const& x)
  {
    Log(error) << "expected: " << x.what_;
    Log(error) << "got: \"" << std::string(x.first, x.last) << '"' << std::endl;

    throw x;
  }

  std::sort(std::begin(instance.get_requests()),
            std::end(instance.get_requests()),
            [](const Request& first, const Request& second) -> bool
            {
              return first.get_time() < second.get_time();
            });

  return instance;
}
