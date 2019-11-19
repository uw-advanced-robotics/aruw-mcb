#include "agitator_rotate_command.hpp"

namespace aruwsrc
{

namespace control
{
    const float AgitatorRotateCommand::agitatorSetpointToleranceDefault = 1.0f;  // degrees

    AgitatorRotateCommand::AgitatorRotateCommand(AgitatorSubsystem* agitator,
        float agitatorAngleChange,
        float setpointTolerance = agitatorSetpointToleranceDefault) :
        aruwlib::control::Command(true),
        agitatorSetpointTolerance(setpointTolerance),
        agitatorTargetChange(agitatorAngleChange)
    {
        this->addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(agitator));
        connectedAgitator = agitator;
    }

    void AgitatorRotateCommand::initialize()
    {
        agitatorStartRotateTime = modm::Clock::now();
        connectedAgitator->setAgitatorAngle(
            connectedAgitator->agitatorEncoderToPosition()
            + agitatorTargetChange
        );
    }

    void AgitatorRotateCommand::execute()
    {}

    void AgitatorRotateCommand::end(bool interrupted)
    {
        if (interrupted)
        {
            connectedAgitator->setAgitatorAngle(
                connectedAgitator->agitatorEncoderToPosition());
        }
    }

    bool AgitatorRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedAgitator->agitatorEncoderToPosition()
            - connectedAgitator->getAgitatorDesiredAngle()))
            < static_cast<double>(agitatorSetpointTolerance);
    }
}  // namespace control

}  // namespace aruwsrc