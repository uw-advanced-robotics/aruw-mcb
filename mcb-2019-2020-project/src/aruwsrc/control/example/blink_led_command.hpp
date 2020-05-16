#ifndef __BLINK_LED_COMMAND_HPP__
#define __BLINK_LED_COMMAND_HPP__

#include <aruwlib/architecture/timeout.hpp>
#include <aruwlib/control/command.hpp>
#include "example_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class BlinkLEDCommand : public Command
{
 public:
    explicit BlinkLEDCommand(aruwsrc::control::ExampleSubsystem* subsystem);

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
    void end(bool);

    /**
      * Whether the command has finished.  Once a command finishes, the scheduler
      * will call its end() method and un-schedule it.
      *
      * @return whether the command has finished.
      */
    bool isFinished(void) const;

    aruwlib::arch::MilliTimeout completedTimer;

    int refershCounter = 0;
    int endCounter = 0;
    int startCounter = 0;
};

}  // namespace control

}  // namespace aruwsrc

#endif
