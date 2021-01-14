#ifndef PROGRAM_HH
#define PROGRAM_HH

#include <string>

#include <objscip/objscip.h>
#include <scip/scip.h>

#include "solution_stats.hh"

class LPObserver;

class Program
{
public:
  struct Settings
  {
    bool solver_output;
    bool collect;

    Settings()
      : solver_output(true),
        collect(false)
    {}

    Settings& with_solver_output(bool output = true)
    {
      solver_output = output;
      return *this;
    }

    Settings& collect_stats(bool doCollect = true)
    {
      collect = doCollect;
      return *this;
    }

  };

private:
  LPObserver* observer;

protected:
  SCIP* scip;

public:
  Program(const std::string& name,
          const Settings& settings = Settings());

  Program(const Program&) = delete;

  Program& operator=(const Program&) = delete;

  SCIP* getSCIP() const
  {
    return scip;
  }

  SolutionStats get_stats() const;

  void add_event_handler(scip::ObjEventhdlr* eventHandler);

  virtual ~Program();

  void write(const std::string& name) const;

  void write_original(const std::string& name) const;
  void write_transformed(const std::string& name) const;

  virtual void init_sol()
  {}

  virtual void exit_sol()
  {}

};

#endif /* PROGRAM_HH */
