#ifndef CMP_HH
#define CMP_HH

#include <cmath>

namespace cmp
{
  extern double eps;

  inline bool eq(double x, double y, double epsval = eps)
  {
    return fabs(x - y) <= epsval;
  }

  inline bool gt(double x, double y, double epsval = eps)
  {
    return ((x) - (y)) > epsval;
  }

  inline bool ge(double x, double y, double epsval = eps)
  {
    return ((x) - (y)) >= -epsval;
  }

  inline bool le(double x, double y, double epsval = eps)
  {
    return ((x) - (y)) <= epsval;
  }

  inline bool lt(double x, double y, double epsval = eps)
  {
    return ((x) - (y)) < -epsval;
  }

  inline bool zero(double x, double epsval = eps)
  {
    return fabs(x) <= epsval;
  }

  inline bool pos(double x, double epsval = eps)
  {
    return (x) > epsval;
  }

  inline bool neg(double x, double epsval = eps)
  {
    return (x) < -epsval;
  }

  inline double floor(double x, double epsval = eps)
  {
    return std::floor(x + epsval);
  }

  inline double ceil(double x, double epsval = eps)
  {
    return std::ceil(x - epsval);
  }

  inline double frac(double x, double epsval = eps)
  {
    return x - floor(x, epsval);
  }

  inline bool integral(double x, double epsval = eps)
  {
    return frac(x,epsval) <= (epsval);
  }

  namespace rel
  {
    inline double diff(double x, double y)
    {
      return (x - y) / std::fmax(std::fmax(std::abs(x), std::abs(y)), 1.);
    }

    inline bool eq(double x, double y, double epsval = eps)
    {
      return cmp::zero(diff(x, y), epsval);
    }

    inline bool gt(double x, double y, double epsval = eps)
    {
      return cmp::pos(diff(x, y), epsval);
    }

    inline bool lt(double x, double y, double epsval = eps)
    {
      return cmp::neg(diff(x, y), epsval);
    }

    inline bool le(double x, double y, double epsval = eps)
    {
      return !cmp::pos(diff(x, y), epsval);
    }

    inline bool ge(double x, double y, double epsval = eps)
    {
      return !cmp::neg(diff(x, y), epsval);
    }

  }

}

#endif /* CMP_HH */
