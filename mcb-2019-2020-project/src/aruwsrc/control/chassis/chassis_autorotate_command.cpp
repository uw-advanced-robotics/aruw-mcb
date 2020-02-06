#include "chassis_autorotate_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwsrc/control/chassis/chassis_subsystem.hpp"

namespace aruwsrc
{

namespace chassis
{

void ChassisAutorotateCommand::initialize()
{}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    float chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(turret->getYawAngleFromCenter(),
        ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR);

    // what we will multiply x and y speed by to take into account rotation
    float rTranslationalGain
        = chassis->calculateRotationTranslationalGain(chassisRotationDesiredWheelspeed);

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

void ChassisAutorotateCommand::end(bool interrupted)
{}

bool ChassisAutorotateCommand::isFinished() const
{
    return false;
}

}  // namespace control

}  // namespace aruwsrc
