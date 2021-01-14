#include "program.hh"

#include <iostream>
#include <fstream>

#include <scip/scipdefplugins.h>

#include "scip_utils.hh"

#include "lp_observer.hh"

namespace
{
  class Notifier : public scip::ObjEventhdlr
  {
  private:
    Program& program;
  public:
    Notifier(Program& program)
      : scip::ObjEventhdlr(program.getSCIP(),
                           "notifier",
                           ""),
        program(program)
    {}

    virtual SCIP_DECL_EVENTEXEC(scip_exec) override
    {
      return SCIP_OKAY;
    }

    virtual SCIP_DECL_EVENTINITSOL(scip_initsol) override
    {
      program.init_sol();
      return SCIP_OKAY;
    }

    virtual SCIP_DECL_EVENTEXITSOL(scip_exitsol) override
    {
      program.exit_sol();
      return SCIP_OKAY;
    }

  };

  inline bool file_exists(const std::string& name)
  {
    std::ifstream f(name.c_str());
    return f.good();
  }

  static SCIP_DECL_MESSAGEWARNING(print_msg)
  {
    if(!msg)
    {
      return;
    }

    if(file == stdout)
    {
      fputs(msg, stderr);
      fflush(stderr);
    }
    else
    {
      fputs(msg, file);
      fflush(file);
    }
  }
}

Program::Program(const std::string& name,
                 const Program::Settings& settings)
  : observer(nullptr)
{
  SCIP_CALL_EXC(SCIPcreate(&scip));
  SCIP_CALL_EXC(SCIPincludeDefaultPlugins(scip));

  {
    std::string fname = BASE_DIRECTORY;
    fname += "/refueling.set";

    if(file_exists(fname))
    {
      SCIP_CALL_EXC(SCIPreadParams(scip, fname.c_str()));
    }
  }

  SCIP_CALL_EXC(SCIPcreateProbBasic(scip, name.c_str()));

  SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip,
                                        new Notifier(*this),
                                        true));

  SCIP_CALL_EXC(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

  if(settings.collect)
  {
    observer = new LPObserver(scip);

    add_event_handler(observer);
  }

  if(settings.solver_output)
  {
    SCIP_MESSAGEHDLR* handler;
    SCIP_CALL_EXC(SCIPmessagehdlrCreate(&handler,
                                        FALSE,
                                        NULL,
                                        FALSE,
                                        print_msg,
                                        print_msg,
                                        print_msg,
                                        NULL,
                                        NULL));

    SCIP_CALL_EXC(SCIPsetMessagehdlr(scip, handler));

    SCIPmessagehdlrRelease(&handler);
  }
  else
  {
    SCIP_CALL_EXC(SCIPsetIntParam(scip,
                                  "display/verblevel",
                                  SCIP_VERBLEVEL_NONE));
  }
}

void Program::add_event_handler(scip::ObjEventhdlr* event_handler)
{
  SCIP_CALL_EXC(SCIPincludeObjEventhdlr(scip,
                                        event_handler,
                                        TRUE));
}

Program::~Program()
{
  SCIP_CALL_ASSERT(SCIPfree(&scip));
}

SolutionStats Program::get_stats() const
{
  if(!observer)
  {
    return SolutionStats::empty();
  }

  return SolutionStats(scip, observer->get_stats());
}

void Program::write(const std::string& name) const
{
  write_original("orig_" + name);
  write_transformed("trans_" + name);
}

void Program::write_original(const std::string& name) const
{
  SCIP_CALL_EXC(SCIPwriteOrigProblem(scip, name.c_str(), nullptr, false));
}

void Program::write_transformed(const std::string& name) const
{
  SCIP_CALL_EXC(SCIPwriteTransProblem(scip, name.c_str(), nullptr, false));
}
