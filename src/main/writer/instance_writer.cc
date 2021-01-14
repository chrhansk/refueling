#include "instance_writer.hh"

void InstanceWriter::write(const Instance& instance,
                           std::ostream& out)
{
  out << "Tankerbaseposition Lat "
      << instance.get_origin().get_lat()
      << " Lon "
      << instance.get_origin().get_lon()
      << std::endl;

  for(const auto& request : instance.get_requests())
  {
    double amount = request.get_amount() / Units::SI::kilogram;

    amount *= Units::conversion_factor(Units::SI::kilogram,
                                       Units::Imperial::Pound::unit_type());

    //const double amount = (request.get_amount() / (Units::Imperial::Pound::unit_type()));
    const double time = (fractional_seconds(request.get_time() - initial_time)  / 3600);

    out << "Request "
        << request.get_index()
        << ", flight "
        << request.get_flight()
        << " at "
        << time
        << " [h] Lat "
        << request.get_origin().get_lat()
        << " Lon "
        << request.get_origin().get_lon()
        << ", flying track "
        << request.get_direction()
        << " for "
        << amount
        << " [lbs] endlat "
        << request.get_destination().get_lat()
        << " endlon "
        << request.get_destination().get_lon()
        << std::endl;
  }
}
