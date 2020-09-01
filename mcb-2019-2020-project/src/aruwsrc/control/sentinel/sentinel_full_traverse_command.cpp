#include "sentinel_full_traverse_command.hpp"

namespace aruwsrc
{
namespace control
{
SentinelFullTraverseCommand::SentinelFullTraverseCommand(SentinelDriveSubsystem* subsystem)
    : prevTime(0),
      velocityTargetGenerator(0),
      subsystemSentinelDrive(subsystem)
{
    addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(subsystem));
}

void SentinelFullTraverseCommand::initialize()
{
    prevTime = aruwlib::arch::clock::getTimeMilliseconds();
    velocityTargetGenerator.reset();
    velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
}

void SentinelFullTraverseCommand::execute()
{
    // reverse direction if close to the end of the rail
    float curPos = subsystemSentinelDrive->absolutePosition();
    if (velocityTargetGenerator.getValue() < 0 && curPos < TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(MAX_DESIRED_TRAVERSE_SPEED);
    }
    else if (
        velocityTargetGenerator.getValue() > 0 &&
        curPos > SentinelDriveSubsystem::RAIL_LENGTH - SentinelDriveSubsystem::SENTINEL_LENGTH -
                     TURNAROUND_BUFFER)
    {
        velocityTargetGenerator.setTarget(-MAX_DESIRED_TRAVERSE_SPEED);
    }
    // update chassis target velocity
    uint32_t currTime = aruwlib::arch::clock::getTimeMilliseconds();
    velocityTargetGenerator.update(RAMP_SPEED * static_cast<float>(currTime - prevTime));
    prevTime = currTime;
    subsystemSentinelDrive->setDesiredRpm(velocityTargetGenerator.getValue());
}

void SentinelFullTraverseCommand::end(bool) { subsystemSentinelDrive->setDesiredRpm(0.0f); }

bool SentinelFullTraverseCommand::isFinished() const { return false; }
}  // namespace control

}  // namespace aruwsrc
