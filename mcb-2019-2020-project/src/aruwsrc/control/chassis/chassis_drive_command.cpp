#include "chassis_drive_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace chassis
{

void ChassisDriveCommand::initialize()
{}

void ChassisDriveCommand::execute()
{
    float chassisRotationDesiredWheelspeed = ChassisSubsystem::getChassisR()
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain = 1.0f;

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the wheel rotation speed for chassis rotationis greater than the
    // MIN_ROTATION_THRESHOLD
    if (fabs(chassisRotationDesiredWheelspeed) > MIN_ROTATION_THRESHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        rTranslationalGain =
            pow(
                (static_cast<double>(
                ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD)
                - fabs(chassisRotationDesiredWheelspeed) )
                / static_cast<double>(ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR),
                2.0
            );
        rTranslationalGain = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }

    float chassisXDesiredWheelspeed =
        aruwlib::algorithms::limitVal<float>(ChassisSubsystem::getChassisX(),
        -rTranslationalGain, rTranslationalGain)
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float chassisYDesiredWheelspeed =
        aruwlib::algorithms::limitVal<float>(ChassisSubsystem::getChassisY(),
        -rTranslationalGain, rTranslationalGain)
        * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;

    chassis->setDesiredOutput(chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed, chassisRotationDesiredWheelspeed);
}

void ChassisDriveCommand::end(bool interrupted)
{
    // if the command was just ended outright, we should set chassis movement to all zeros
    // if the command was interrupted, however, we know that another command is running so
    // we don't need to change the output
    if (!interrupted)
    {
        chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
    }
}

bool ChassisDriveCommand::isFinished() const
{
    return false;
}

}  // namespace chassis

}  // namespace aruwsrc
