#include "chassis_autorotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{

void ChassisAutorotateCommand::initialize()
{}

void ChassisAutorotateCommand::execute()
{
    float remoteMoveX = aruwlib::Remote::getChassisX();
    float remoteMoveY = aruwlib::Remote::getChassisY();

    float chassisMoveX, chassisMoveY, chassisMoveZ;
    float turretRelativeX, turretRelativeY;
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    chassisMoveZ = chassis->chassisSpeedZPID(turret->gimbalGetOffset(), ChassisSubsystem::CHASSIS_AUTOROTATE_PID_KP);

    float zTranslationGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the rotation speed is greater than the MIN_ROTATION_THREASHOLD
    if (fabs(chassisMoveZ) > MIN_ROTATION_THREASHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2)
        zTranslationGain = 
            pow( (static_cast<double>(ChassisSubsystem::OMNI_SPEED_MAX + MIN_ROTATION_THREASHOLD) - fabs(chassisMoveZ) )
            / static_cast<double>(ChassisSubsystem::OMNI_SPEED_MAX), 2.0);
        
        zTranslationGain = aruwlib::algorithms::limitVal<float>(zTranslationGain, 0.0f, 1.0f);
    }
    else
    {
		zTranslationGain = 1.0f;
    }

    // translate x and y relative to turret for turret relative control
    // translation matrix:
    // translate the controls by angle theta defined by the gimbal center offset
    double gimbalOffsetRad = -static_cast<double>(DEGREES_TO_RADIANS(turret->gimbalGetOffset()));
    turretRelativeX = remoteMoveX * static_cast<float>(cos(gimbalOffsetRad))
        - remoteMoveY * static_cast<float>(sin(gimbalOffsetRad));
    turretRelativeY = remoteMoveX * static_cast<float>(sin(gimbalOffsetRad))
        + remoteMoveY * static_cast<float>(cos(gimbalOffsetRad));

	chassisMoveX = aruwlib::algorithms::limitVal<float>(turretRelativeX,
        -zTranslationGain, zTranslationGain);
	chassisMoveY = aruwlib::algorithms::limitVal<float>(turretRelativeY,
        -zTranslationGain, zTranslationGain);

    chassis->setDesiredOutput(chassisMoveX * ChassisSubsystem::OMNI_SPEED_MAX,
        chassisMoveY * ChassisSubsystem::OMNI_SPEED_MAX, chassisMoveZ);
}

void ChassisAutorotateCommand::end(bool interrupted)
{
    if (interrupted) {}
}

bool ChassisAutorotateCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
