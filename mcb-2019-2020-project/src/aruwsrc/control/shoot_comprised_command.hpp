#ifndef __SHOOT_COMPRISED_COMMAND_HPP__
#define __SHOOT_COMPRISED_COMMAND_HPP__

#include "src/control/command.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class ShootComprisedCommand : public aruwlib::control::Command
{
public:
    ShootComprisedCommand(
        AgitatorSubsystem* agitator,
        float agitatorChangeAngle,
        float maxUnjamAngle
    );

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
    AgitatorSubsystem* connectedAgitator; 

    modm::SmartPointer agitatorRotateCommand;

    modm::SmartPointer agitatorUnjamCommand;

    bool unjamSequenceCommencing;
};

}  // namespace control

}  // namespace aruwsrc

#endif
