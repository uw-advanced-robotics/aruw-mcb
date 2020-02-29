#include "wrist_rotate_command.hpp"

namespace aruwsrc
{

namespace engineer
{
    WristRotateCommand::WristRotateCommand(
        WristSubsystem* wrist,
        float wristAngleChange,
        float wristRotateTime) :
        wristTargetChange(wristAngleChange),
        wristRotateSetpointLeft(
            WRIST_ROTATE_COMMAND_PERIOD * fabs(wristAngleChange) / wristRotateTime,
            WRIST_ROTATE_COMMAND_PERIOD * fabs(wristAngleChange) / wristRotateTime, 0),
        wristRotateSetpointRight(
            WRIST_ROTATE_COMMAND_PERIOD * fabs(wristAngleChange) / wristRotateTime,
            WRIST_ROTATE_COMMAND_PERIOD * fabs(wristAngleChange) / wristRotateTime, 0),
        wristDesiredRotateTime(wristRotateTime),
        wristMinRotateTime(WRIST_MIN_ROTATE_TIME)
    {
        this->addSubsystemRequirement(dynamic_cast<aruwlib::control::Subsystem*>(wrist));
        connectedWrist = wrist;
    }

    void WristRotateCommand::initialize()
    {
        wristRotateSetpointLeft.reset(connectedWrist->getWristAngleLeft());
        wristRotateSetpointRight.reset(connectedWrist->getWristAngleRight());

        if ((wristTargetChange > 0 && connectedWrist->isIn()) || (wristTargetChange < 0 && !connectedWrist->isIn()))
        {
            connectedWrist->wristToggleState();
            wristRotateSetpointLeft.setTarget(connectedWrist->getWristDesiredAngleLeft() + wristTargetChange);
            wristRotateSetpointRight.setTarget(connectedWrist->getWristDesiredAngleRight() + wristTargetChange);
        } else {
            wristRotateSetpointLeft.setTarget(connectedWrist->getWristDesiredAngleLeft());
            wristRotateSetpointRight.setTarget(connectedWrist->getWristDesiredAngleRight());
        }

        wristMinRotateTime.restart(WRIST_MIN_ROTATE_TIME);
    }

    void WristRotateCommand::execute()
    {
        wristRotateSetpointLeft.update();
        wristRotateSetpointRight.update();

        connectedWrist->setWristAngleLeft(wristRotateSetpointLeft.getValue());
        connectedWrist->setWristAngleRight(wristRotateSetpointRight.getValue());
    }

    void WristRotateCommand::end(bool interrupted)
    {
        connectedWrist->setWristAngleLeft(connectedWrist->getWristDesiredAngleLeft());
        connectedWrist->setWristAngleRight(connectedWrist->getWristDesiredAngleRight());
    }

    // Finished if the left and right side of the wrist has reached target within the tolerance
    bool WristRotateCommand::isFinished() const
    {
        return fabs(static_cast<double>(connectedWrist->getWristAngleLeft()
         - connectedWrist->getWristDesiredAngleLeft()))
         < static_cast<double>(WRIST_SETPOINT_TOLERANCE)
         && fabs(static_cast<double>(connectedWrist->getWristAngleRight()
         - connectedWrist->getWristDesiredAngleRight()))
         < static_cast<double>(WRIST_SETPOINT_TOLERANCE)
         && wristRotateSetpointLeft.isTargetReached()
         && wristRotateSetpointRight.isTargetReached()
         && wristMinRotateTime.isExpired();
    }
}  // namespace engineer

}  // namespace aruwsrc