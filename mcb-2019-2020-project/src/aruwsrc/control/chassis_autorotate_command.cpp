#include "chassis_autorotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"

namespace aruwsrc
{

namespace control
{

int16_t gimbalGetOffset(void) {
    return 0;
}

void ChassisAutorotateCommand::initialize()
{

}

void ChassisAutorotateCommand::execute()
{
    float chassisMoveX, chassisMoveY, chassisMoveZ;
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    chassisMoveZ = chassis->chassisSpeedZPID(gimbalGetOffset(), ChassisSubsystem::CHASSIS_AUTOROTATE_PID_KP);

    float zTranslationGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption when the rotation speed is greater than the MIN_ROTATION_THREASHOLD
    if (fabs(chassisMoveZ) > MIN_ROTATION_THREASHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2) (don't use double operation)
        zTranslationGain =
            pow((ChassisSubsystem::OMNI_SPEED_MAX - fabs(chassisMoveZ) + MIN_ROTATION_THREASHOLD)
            / ( ChassisSubsystem::OMNI_SPEED_MAX * ChassisSubsystem::OMNI_SPEED_MAX), 2.0);
        
        zTranslationGain = aruwlib::algorithms::limitVal<float>(zTranslationGain, 0.0f, 1.0f);
    }
    else
    {
		zTranslationGain = 1;
    }

	chassisMoveX = aruwlib::algorithms::limitVal<float>(0/**/,
        -ChassisSubsystem::OMNI_SPEED_MAX * zTranslationGain,
         ChassisSubsystem::OMNI_SPEED_MAX * zTranslationGain);
	chassisMoveY = aruwlib::algorithms::limitVal<float>(0,
        -ChassisSubsystem::OMNI_SPEED_MAX * zTranslationGain,
         ChassisSubsystem::OMNI_SPEED_MAX * zTranslationGain);

    chassis->setDesiredOutput(chassisMoveX, chassisMoveY, chassisMoveZ);
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
