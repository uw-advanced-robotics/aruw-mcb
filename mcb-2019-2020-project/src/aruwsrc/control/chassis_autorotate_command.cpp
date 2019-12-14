#include "chassis_autorotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"

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
    chassisMoveZ = chassis->Chassis_SpeedZ_PID(gimbalGetOffset(), chassisAutorotateKp);

    float zTranslationGain;  // what we will multiply x and y speed by to take into account rotation

    // the x and y movement will be slowed by a fraction of auto rotation amount for maximizing
    // power consumption
    if (fabs(chassisMoveZ) > MIN_ROTATION_THREASHOLD)
    {
        // power(max revolve speed - specified revolve speed, 2)
        // / power(max revolve speed, 2) (don't use double operation)
        zTranslationGain = (
            (ChassisSubsystem::REVOLVE_MAX_NORMAL - fabs(chassisMoveZ) + MIN_ROTATION_THREASHOLD)
            * (ChassisSubsystem::REVOLVE_MAX_NORMAL - fabs(chassisMoveZ) + MIN_ROTATION_THREASHOLD))
            / ( ChassisSubsystem::REVOLVE_MAX_NORMAL * ChassisSubsystem::REVOLVE_MAX_NORMAL);
        
        zTranslationGain = aruwlib::algorithms::limitVal<float>(zTranslationGain, 0.0f, 1.0f);
    }
    else
    {
		zTranslationGain = 1;
    }

	chassisMoveX = aruwlib::algorithms::limitVal<float>(0 /**/,
        -ChassisSubsystem::STANDARD_MAX_NORMAL * zTranslationGain,
         ChassisSubsystem::STANDARD_MAX_NORMAL * zTranslationGain);
	chassisMoveY = aruwlib::algorithms::limitVal<float>(0,
        -ChassisSubsystem::STANDARD_MAX_NORMAL * zTranslationGain,
         ChassisSubsystem::STANDARD_MAX_NORMAL * zTranslationGain);

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
