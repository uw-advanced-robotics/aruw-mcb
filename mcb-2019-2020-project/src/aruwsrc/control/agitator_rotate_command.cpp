#include "agitator_rotate_command.hpp"

namespace aruwsrc
{

namespace control
{
    const float AgitatorRotateCommand::agitatorSetpointToleranceDefault = 1.0f;

    const float AgitatorRotateCommand::agitatorRampInc = 0.01f;

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
        agitatorRotateSetpoint.reset(connectedAgitator->agitatorEncoderToPosition());
        agitatorRotateSetpoint.setTarget(connectedAgitator->agitatorEncoderToPosition()
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
            connectedAgitator->setAgitatorAngle(connectedAgitator->agitatorEncoderToPosition());
        }
        connectedAgitator->disarmAgitatorUnjamTimer();
    }

    bool AgitatorRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedAgitator->agitatorEncoderToPosition()
            - connectedAgitator->getAgitatorDesiredAngle()))
            < static_cast<double>(agitatorSetpointTolerance)
            && agitatorRotateSetpoint.isTargetReached();
    }
}  // namespace control

}  // namespace aruwsrc