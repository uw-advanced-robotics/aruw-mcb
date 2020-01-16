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
    float remoteMoveX = ChassisSubsystem::getChassisX();
    float remoteMoveY = ChassisSubsystem::getChassisY();
    float remoteMoveZ = ChassisSubsystem::getChassisR();

    float chassisMoveX, chassisMoveY, chassisMoveZ;

    chassisMoveZ = remoteMoveZ * ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR;

    float rTranslationalGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the rotation speed is greater than the MIN_ROTATION_THRESHOLD
    if (fabs(chassisMoveZ) > MIN_ROTATION_THRESHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        rTranslationalGain =
            pow(
                (static_cast<double>(
                ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR + MIN_ROTATION_THRESHOLD)
                - fabs(chassisMoveZ) )
                / static_cast<double>(ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR),
                2.0
            );
        rTranslationalGain = aruwlib::algorithms::limitVal<float>(rTranslationalGain, 0.0f, 1.0f);
    }
    else
    {
        rTranslationalGain = 1.0f;
    }

    chassisMoveX = aruwlib::algorithms::limitVal<float>(remoteMoveX,
        -rTranslationalGain, rTranslationalGain);
    chassisMoveY = aruwlib::algorithms::limitVal<float>(remoteMoveY,
        -rTranslationalGain, rTranslationalGain);

    chassis->setDesiredOutput(chassisMoveX * ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR,
        chassisMoveY * ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR, chassisMoveZ);
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
