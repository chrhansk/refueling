#ifndef INSTANCE_WRITER_HH
#define INSTANCE_WRITER_HH

#include <iostream>

#include "instance.hh"

class InstanceWriter
{
public:
  void write(const Instance& instance,
             std::ostream& out);
};


#endif /* INSTANCE_WRITER_HH */
