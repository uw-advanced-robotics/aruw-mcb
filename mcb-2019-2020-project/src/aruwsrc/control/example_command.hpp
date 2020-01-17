/**
 * This code is part of aruw's repository
 * 
 * Example code for a default command for the subsystem-example subsystem.
 */

#ifndef __COMMAND_EXAMPLE_HPP__
#define __COMMAND_EXAMPLE_HPP__

#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ExampleSubsystem;

class ExampleCommand : public Command
{
 public:
    explicit ExampleCommand(ExampleSubsystem* subsystem);

    /**
      * The initial subroutine of a command.  Called once when the command is
      * initially scheduled.
      */
    void initialize(void);

    /**
      * The main body of a command.  Called repeatedly while the command is
      * scheduled.
      */
    void execute(void);

    /**
      * The action to take when the command ends.  Called when either the command
      * finishes normally, or when it interrupted/canceled.
      *
      * @param interrupted whether the command was interrupted/canceled
      */
    void end(bool interrupted);

    /**
      * Whether the command has finished.  Once a command finishes, the scheduler
      * will call its end() method and un-schedule it.
      *
      * @return whether the command has finished.
      */
    bool isFinished(void) const;

    void interrupted(void);

 private:
    static const int16_t DEFAULT_WHEEL_RPM = 6000;

    ExampleSubsystem* subsystemExample;
};

}  // namespace control

}  // namespace aruwsrc
#endif
