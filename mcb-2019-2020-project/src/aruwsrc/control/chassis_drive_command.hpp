#ifndef __CHASSIS_DRIVE_COMMAND_HPP__
#define __CHASSIS_DRIVE_COMMAND_HPP__

#include "src/aruwlib/control/command.hpp"
#include "chassis_subsystem.hpp"

using namespace aruwlib::control;

namespace aruwsrc
{

namespace control
{

class ChassisDriveCommand : public Command {
public:
    explicit ChassisDriveCommand(ChassisSubsystem* chassis = nullptr);

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
    
private:
    ChassisSubsystem* chassis;
};

}  // namespace control

}  // namespace aruwsrc

#endif