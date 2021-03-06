#include "EngineerWristRotateCommand.hpp"

#include "EngineerWristSubsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
EngineerWristRotateCommand::EngineerWristRotateCommand(
    EngineerWristSubsystem* wrist,
    float wristAngle,
    float wristRotateTime)
    : connectedWrist(wrist),
      wristAngle(wristAngle),
      wristRotateSetpoint(0),
      wristRotateSetpointRight(0),
      wristDesiredRotateTime(wristRotateTime),
      wristMinRotateTime(WRIST_MIN_ROTATE_TIME)
{
    addSubsystemRequirement(reinterpret_cast<aruwlib::control::Subsystem*>(wrist));
    connectedWrist = wrist;
}

void EngineerWristRotateCommand::initialize()
{
    // wristRotateSetpoint.reset(connectedWrist->getWristAngle()); TODO reset
    wristRotateSetpoint.setTarget(wristAngle);
    wristMinRotateTime.restart(WRIST_MIN_ROTATE_TIME);
}

void EngineerWristRotateCommand::execute()
{
    float increment = 0.0f;  // TODO
    wristRotateSetpoint.update(increment);
    connectedWrist->setWristAngle(wristRotateSetpoint.getValue());
}

void EngineerWristRotateCommand::end(bool) { connectedWrist->setWristAngle(wristAngle); }

// Finished if the left and right side of the wrist has reached target within the tolerance
bool EngineerWristRotateCommand::isFinished() const
{
    return wristRotateSetpoint.isTargetReached() && wristMinRotateTime.isExpired();
}
}  // namespace engineer
}  // namespace aruwsrc
