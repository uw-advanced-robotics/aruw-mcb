#include "agitator_rotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

namespace aruwsrc
{

namespace control
{
    const float AgitatorRotateCommand::agitatorSetpointToleranceDefault = aruwlib::algorithms::PI / 16.0f;

    const float AgitatorRotateCommand::AGITATOR_RAMP_INC = 0.07f;

    AgitatorRotateCommand::AgitatorRotateCommand(AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float setpointTolerance,
        float agitatorAngleIncrement) :
        agitatorSetpointTolerance(setpointTolerance),
        agitatorTargetChange(agitatorAngleChange),
        agitatorRotateSetpoint(agitatorAngleIncrement, agitatorAngleIncrement, 0)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(agitator));
        connectedAgitator = agitator;
    }

    void AgitatorRotateCommand::initialize()
    {
        agitatorRotateSetpoint.reset(connectedAgitator->getAgitatorEncoderToPosition());
        agitatorRotateSetpoint.setTarget(connectedAgitator->getAgitatorEncoderToPosition()
            + agitatorTargetChange);
        connectedAgitator->armAgitatorUnjamTimer();
    }

    void AgitatorRotateCommand::execute()
    {
        agitatorRotateSetpoint.update();
        connectedAgitator->setAgitatorAngle(agitatorRotateSetpoint.getValue());
    }

    void AgitatorRotateCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            connectedAgitator->setAgitatorAngle(connectedAgitator->getAgitatorEncoderToPosition());
        }
        connectedAgitator->disarmAgitatorUnjamTimer();
    }

    bool AgitatorRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedAgitator->getAgitatorEncoderToPosition()
            - connectedAgitator->getAgitatorDesiredAngle()))
            < static_cast<double>(agitatorSetpointTolerance)
            && agitatorRotateSetpoint.isTargetReached();
    }
}  // namespace control

}  // namespace aruwsrc