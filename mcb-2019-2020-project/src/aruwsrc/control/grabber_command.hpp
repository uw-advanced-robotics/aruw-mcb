/**
 * This code is part of aruw's repository
 * this is Engineer grabber mechanism's command code
 * 
 */

#ifndef __COMMAND_GRABBER_HPP__
#define __COMMAND_GRABBER_HPP__

#include "src/aruwlib/control/command.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class GrabberSubsystem;

class GrabberCommand : public Command
{
 public:
    explicit GrabberCommand(GrabberSubsystem* subsystem = nullptr);

    /**
      * The initial subroutine of a command.  Called once when the command is
      * initially scheduled.
      */
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
    GrabberSubsystem* subsystemGrabber;
    
};

}  // namespace control

}  // namespace aruwsrc
#endif
