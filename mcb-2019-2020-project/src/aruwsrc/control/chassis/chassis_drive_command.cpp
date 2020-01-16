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
    float remoteMoveZ = ChassisSubsystem::getChassisZ();

    float chassisMoveX, chassisMoveY, chassisMoveZ;

    chassisMoveZ = remoteMoveZ * ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR;

    float zTranslationGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the rotation speed is greater than the MIN_ROTATION_THREASHOLD
    if (fabs(chassisMoveZ) > MIN_ROTATION_THREASHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        zTranslationGain =
            pow(
                (static_cast<double>(
                ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR + MIN_ROTATION_THREASHOLD)
                - fabs(chassisMoveZ) )
                / static_cast<double>(ChassisSubsystem::MAX_CURRENT_OUT_SINGLE_MOTOR),
                2.0
            );
        zTranslationGain = aruwlib::algorithms::limitVal<float>(zTranslationGain, 0.0f, 1.0f);
    }
    else
    {
        zTranslationGain = 1.0f;
    }

    chassisMoveX = aruwlib::algorithms::limitVal<float>(remoteMoveX,
        -zTranslationGain, zTranslationGain);
    chassisMoveY = aruwlib::algorithms::limitVal<float>(remoteMoveY,
        -zTranslationGain, zTranslationGain);

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
