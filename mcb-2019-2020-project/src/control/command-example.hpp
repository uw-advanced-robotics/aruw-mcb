/**
 * This code is part of aruw's repository
 * 
 * Example code for a default command for the subsystem-example subsystem.
 */

#ifndef __COMMAND_EXAMPLE_HPP__
#define __COMMAND_EXAMPLE_HPP__

#include "src/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{
class SubsystemExample;

class CommandExample : public Command
{
 public:
    explicit CommandExample(SubsystemExample* subsystem)
    {
        addSubsystemRequirement(reinterpret_cast<Subsystem*>(subsystem));
        subsystemExample = subsystem;
    }

    ~CommandExample()
    {
    }

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
    bool isFinished(void);

    void interrupted(void);

    /**
      * Whether the given command should run when the robot is disabled.  Override
      * to return true if the command should run when disabled.
      *
      * @return whether the command should run when the robot is disabled
      */
    bool runsWhenDisabled(void) const;
 private:
    static const int16_t DEFAULT_WHEEL_RPM = 5000;

    SubsystemExample* subsystemExample;
};

}  // namespace control

}  // namespace aruwsrc
#endif
