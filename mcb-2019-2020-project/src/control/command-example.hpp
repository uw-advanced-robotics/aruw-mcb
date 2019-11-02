#ifndef __COMMAND_EXAMPLE_HPP__
#define __COMMAND_EXAMPLE_HPP__

#include "src/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class CommandExample : public Command
{
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

    /**
      * Specifies the set of subsystems used by this command.  Two commands cannot
      * use the same subsystem at the same time.  If the command is scheduled as
      * interruptible and another command is scheduled that shares a requirement,
      * the command will be interrupted.  Else, the command will not be scheduled.
      * If no subsystems are required, return an empty set.
      *
      * <p>Note: it is recommended that user implementations contain the
      * requirements as a field, and return that field here, rather than allocating
      * a new set every time this is called.
      *
      * @return the set of subsystems that are required
      */
    modm::LinkedList<Subsystem*> getRequirements(void) const; //<- pointer stuff rough for now

    void interrupted(void)
    {}

    /**
      * Whether the given command should run when the robot is disabled.  Override
      * to return true if the command should run when disabled.
      *
      * @return whether the command should run when the robot is disabled
      */
    bool runsWhenDisabled(void) const;
};

}  // namespace control

}  // namespace aruwsrc
#endif