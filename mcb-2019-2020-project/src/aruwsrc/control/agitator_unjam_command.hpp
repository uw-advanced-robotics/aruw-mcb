#ifndef __AGITATOR_UNJAM_COMMAND_HPP__
#define __AGITATOR_UNJAM_COMMAND_HPP__

#include <modm/processing/timer/timeout.hpp>
#include "src/algorithms/math_user_utils.hpp"
#include "src/control/command.hpp"
#include "src/motor/dji_motor.hpp"
#include "agitator_subsystem.hpp"

namespace aruwsrc
{

namespace control
{

class AgitatorUnjamCommand : public aruwlib::control::Command
{
public:
    AgitatorUnjamCommand(
        AgitatorSubsystem* agitator,
        float agitatorMaxUnjamAngle,
        uint32_t agitatorMaxWaitTime = AGITATOR_MAX_WAIT_TIME,
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

    static const uint32_t AGITATOR_MAX_WAIT_TIME;  // in ms

    static const float MIN_AGITATOR_UNJAM_ANGLE;

    enum AgitatorUnjamState {AGITATOR_UNJAM_BACK, AGITATOR_UNJAM_FORWARD, FINISHED};

    AgitatorUnjamState currUnjamstate;

    // time allowed to rotate back the the currAgitatorUnjamAngle
    modm::ShortTimeout agitatorUnjamRotateTimeout;

    uint32_t agitatorMaxWaitTime;

    AgitatorSubsystem* connectedAgitator;

    float agitatorSetpointTolerance;

    float agitatorUnjamAngleMax;

    float currAgitatorUnjamAngle;

    float agitatorSetpointBeforeUnjam;
};

}  // namespace control

}  // namespace aruwsrc

#endif
