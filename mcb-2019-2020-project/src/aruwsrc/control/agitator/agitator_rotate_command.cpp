#include "agitator_rotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace agitator
{
    AgitatorRotateCommand::AgitatorRotateCommand(
        AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float agitatorRotateTime,
        float agitatorPauseAfterRotateTime) :
        agitatorTargetChange(agitatorAngleChange),
        agitatorRotateSetpoint(
            AGITATOR_ROTATE_COMMAND_PERIOD * agitatorAngleChange / agitatorRotateTime,
            AGITATOR_ROTATE_COMMAND_PERIOD * agitatorAngleChange / agitatorRotateTime, 0),
        agitatorDesiredRotateTime(agitatorRotateTime),
        agitatorMinRotatePeriod(agitatorRotateTime + agitatorPauseAfterRotateTime),
        agitatorMinRotateTimeout(agitatorRotateTime + agitatorPauseAfterRotateTime)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(agitator));
        connectedAgitator = agitator;
    }

    void AgitatorRotateCommand::initialize()
    {
        // set the ramp start and target angles
        agitatorRotateSetpoint.reset(connectedAgitator->getAgitatorAngle());
        agitatorRotateSetpoint.setTarget(connectedAgitator->getAgitatorAngle()
            + agitatorTargetChange);
        
        // we set the unjam timer to the larger of two values:
        // either the desired rotate time minimum rotate time
        connectedAgitator->armAgitatorUnjamTimer(
            agitatorDesiredRotateTime > agitatorMinRotatePeriod ?
            agitatorDesiredRotateTime : agitatorMinRotatePeriod);
        agitatorMinRotateTimeout.restart(agitatorMinRotatePeriod);
    }

    void AgitatorRotateCommand::execute()
    {
        // update the agitator setpoint ramp
        agitatorRotateSetpoint.update();
        connectedAgitator->setAgitatorAngle(agitatorRotateSetpoint.getValue());
    }

    void AgitatorRotateCommand::end(bool interrupted)
    {
        // if the agitator is not interrupted, then it exited normally
        // (i.e. reached the desired angle) and is not jammed. If it is
        // jammed we thus want to set the agitator angle to the current angle,
        // so the motor does not attempt to keep rotating forward (and possible stalling)
        if (connectedAgitator->isAgitatorJammed())
        {
            connectedAgitator->setAgitatorAngle(connectedAgitator->getAgitatorAngle());
        }
        connectedAgitator->disarmAgitatorUnjamTimer();
    }

    bool AgitatorRotateCommand::isFinished() const
    {
        // The agitator is within the setpoint tolerance, the agitator ramp is
        // finished, and the minimum rotate time is expired.
        return fabs(static_cast<double>(connectedAgitator->getAgitatorAngle()
            - connectedAgitator->getAgitatorDesiredAngle()))
            < static_cast<double>(AGITATOR_SETPOINT_TOLERANCE)
            && agitatorRotateSetpoint.isTargetReached()
            && agitatorMinRotateTimeout.isExpired();  // agitator min timeout finished
    }
}  // namespace control

}  // namespace aruwsrc