#ifndef __AGITATOR_ROTATE_COMMAND_HPP__
#define __AGITATOR_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include "src/control/command.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorRotateCommand : public aruwlib::control::Command
{
 public:
    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float setpointTolerance
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
    modm::Timestamp agitatorStartRotateTime;

    AgitatorSubsystem* connectedAgitator;

    static const float agitatorSetpointToleranceDefault;  // degrees

    float agitatorSetpointTolerance;

    float agitatorTargetChange;
};

}  // namespace control

}  // namespace aruwsrc

#endif