#include "sentinel_full_traverse_command.hpp"
#include "sentinel_drive_subsystem.hpp"

namespace aruwsrc
{

namespace control
{
    SentinelFullTraverseCommand::SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem) :
        velocityTargetGenerator(0),
        subsystemSentinelDrive(subsystem)
    {
        addSubsystemRequirement(dynamic_cast<Subsystem*>(subsystem));
    }

    void SentinelFullTraverseCommand::initialize()
    {
        prevTime = modm::Clock::now().getTime();
        velocityTargetGenerator.reset();
        velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
    }

    void SentinelFullTraverseCommand::execute()
    {
        uint32_t currTime = modm::Clock::now().getTime();
        velocityTargetGenerator.update(RAMP_SPEED * static_cast<float>(currTime - prevTime));
        prevTime = currTime;
        // reverse direction if close to the end of the rail
        float curPos = subsystemSentinelDrive->absolutePosition();
        if (velocityTargetGenerator.getValue() < 0 && curPos < RAIL_BUFFER)
        {
            velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
        }
        else if (velocityTargetGenerator.getValue() > 0 && curPos
                > SentinelDriveSubsystem::RAIL_LENGTH - RAIL_BUFFER)
        {
            velocityTargetGenerator.setTarget(-MAX_DESIRED_TRAVERSE_SPEED);
        }
        subsystemSentinelDrive->setDesiredRpm(velocityTargetGenerator.getValue());
    }

    // NOLINTNEXTLINE
    void SentinelFullTraverseCommand::end(bool)
    {
        subsystemSentinelDrive->setDesiredRpm(0.0f);
    }

    bool SentinelFullTraverseCommand::isFinished() const
    {
        return false;
    }
}  // namespace control

}  // namespace aruwsrc
