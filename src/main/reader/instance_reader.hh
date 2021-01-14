#ifndef INSTANCE_READER_HH
#define INSTANCE_READER_HH

#include <iostream>

#include "instance.hh"

class InstanceReader
{
public:
  Instance read(std::istream& input);
};

#endif /* INSTANCE_READER_HH */
