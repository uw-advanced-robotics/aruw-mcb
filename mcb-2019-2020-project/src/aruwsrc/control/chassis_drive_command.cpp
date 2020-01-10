#include "chassis_drive_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{

void ChassisDriveCommand::initialize()
{}

void ChassisDriveCommand::execute()
{
    float remoteMoveX = aruwlib::Remote::getChassisX();
    float remoteMoveY = aruwlib::Remote::getChassisY();
    float remoteMoveZ = aruwlib::Remote::getChassisZ();

    float chassisMoveX, chassisMoveY, chassisMoveZ;
    
    chassisMoveZ = remoteMoveZ * ChassisSubsystem::OMNI_SPEED_MAX;

    float zTranslationGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the rotation speed is greater than the MIN_ROTATION_THREASHOLD
    if (fabs(chassisMoveZ) > MIN_ROTATION_THREASHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        zTranslationGain =
            pow(
                (static_cast<double>(ChassisSubsystem::OMNI_SPEED_MAX + MIN_ROTATION_THREASHOLD)
                - fabs(chassisMoveZ) )
                / static_cast<double>(ChassisSubsystem::OMNI_SPEED_MAX),
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

    chassis->setDesiredOutput(chassisMoveX * ChassisSubsystem::OMNI_SPEED_MAX,
        chassisMoveY * ChassisSubsystem::OMNI_SPEED_MAX, chassisMoveZ);
}

void ChassisDriveCommand::end(bool interrupted)
{}

bool ChassisDriveCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
