#ifndef __AGITATOR_ROTATE_COMMAND_HPP__
#define __AGITATOR_ROTATE_COMMAND_HPP__

#include <modm/math/filter/pid.hpp>
#include <modm/math/filter/ramp.hpp>
#include "src/aruwlib/control/command.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorRotateCommand : public aruwlib::control::Command
{
 public:
    static const uint32_t AGITATOR_MIN_ROTATE_TIME = 200;

    AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float agitatorAngleIncrement = AGITATOR_RAMP_INC,
        float setpointTolerance = agitatorSetpointToleranceDefault
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
    static const float agitatorSetpointToleranceDefault;

    static const float AGITATOR_RAMP_INC;  // in radians

    AgitatorSubsystem* connectedAgitator;

    float agitatorSetpointTolerance;

    float agitatorTargetChange;

    modm::filter::Ramp<float> agitatorRotateSetpoint;

    modm::ShortTimeout agitatorMinRotateTime;
};

}  // namespace control

}  // namespace aruwsrc

#endif