#ifndef UTIL_HH
#define UTIL_HH

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <memory>
#include <random>
#include <vector>
#include <unordered_map>

#include "cmp.hh"
#include "defs.hh"
#include "point.hh"

typedef uint32_t idx;
typedef int32_t num;
typedef unsigned int uint;

extern num inf;

/**
 * Returns whether the debug mode is enabled.
 **/
constexpr bool debugging_enabled()
{
#if defined(NDEBUG)
  return false;
#else
  return true;
#endif
}

namespace std
{
  template <class T>
  inline void hash_combination(std::size_t & seed, const T & v)
  {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
  }

  template<typename T, typename ...Args>
  unique_ptr<T> make_unique( Args&& ...args )
  {
    return unique_ptr<T>( new T( std::forward<Args>(args)... ) );
  }

  template <>
  struct hash<Point>
  {
    typedef std::size_t result_type;

    result_type operator()(const Point& p) const
    {
      result_type seed = 0;
      std::hash_combination(seed, std::hash<double>{}(p.get_lat()));
      std::hash_combination(seed, std::hash<double>{}(p.get_lon()));

      return seed;
    }
  };
}

inline double deg_to_rad(const double deg)
{
  return deg * (M_PI / (180.0));
}

#endif /* UTIL_HH */
