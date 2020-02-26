#include "agitator_rotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace agitator
{
    AgitatorRotateCommand::AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        uint32_t agitatorRotateTime,
        uint32_t agitatorPauseAfterRotateTime,
        float setpointTolerance
    ) :
        connectedAgitator(agitator),
        agitatorTargetAngleChange(agitatorAngleChange),
        rampToTargetAngle(0.0f),
        agitatorDesiredRotateTime(agitatorRotateTime),
        agitatorMinRotatePeriod(agitatorRotateTime + agitatorPauseAfterRotateTime),
        agitatorMinRotateTimeout(agitatorRotateTime + agitatorPauseAfterRotateTime),
        agitatorSetpointTolerance(setpointTolerance),
        agitatorPrevRotateTime(0)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
    }

    void AgitatorRotateCommand::initialize()
    {
        // set the ramp start and target angles
        rampToTargetAngle.setTarget(connectedAgitator->getAgitatorDesiredAngle()
            + agitatorTargetAngleChange);

        // we set the unjam timer to the larger of two values:
        // either the desired rotate time minimum rotate time
        connectedAgitator->armAgitatorUnjamTimer(agitatorMinRotatePeriod);
        agitatorMinRotateTimeout.restart(agitatorMinRotatePeriod);

        agitatorPrevRotateTime = modm::Clock::now().getTime();
    }

    void AgitatorRotateCommand::execute()
    {
        // update the agitator setpoint ramp
        uint32_t currTime = modm::Clock::now().getTime();
        rampToTargetAngle.update(
                (currTime - agitatorPrevRotateTime) * agitatorTargetAngleChange
                / static_cast<float>(agitatorDesiredRotateTime));
        agitatorPrevRotateTime = modm::Clock::now().getTime();
        connectedAgitator->setAgitatorDesiredAngle(rampToTargetAngle.getValue());
    }

    // NOLINTNEXTLINE
    void AgitatorRotateCommand::end(bool)
    {
        // if the agitator is not interrupted, then it exited normally
        // (i.e. reached the desired angle) and is not jammed. If it is
        // jammed we thus want to set the agitator angle to the current angle,
        // so the motor does not attempt to keep rotating forward (and possible stalling)
        if (connectedAgitator->isAgitatorJammed())
        {
            connectedAgitator->setAgitatorDesiredAngle(connectedAgitator->getAgitatorAngle());
        }
        else
        {
            // connectedAgitator->setAgitatorDesiredAngle(rampToTargetAngle.getTarget());
        }
        connectedAgitator->setAgitatorDesiredAngle(connectedAgitator->getAgitatorAngle());
        connectedAgitator->disarmAgitatorUnjamTimer();
    }

    bool AgitatorRotateCommand::isFinished() const
    {
        // The agitator is within the setpoint tolerance, the agitator ramp is
        // finished, and the minimum rotate time is expired.
        return fabsf(connectedAgitator->getAgitatorAngle()
                - connectedAgitator->getAgitatorDesiredAngle())
                < agitatorSetpointTolerance
                && agitatorMinRotateTimeout.isExpired();
    }
}  // namespace agitator

}  // namespace aruwsrc
