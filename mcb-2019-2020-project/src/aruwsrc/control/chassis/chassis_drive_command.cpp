#include "chassis_drive_command.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"

namespace aruwsrc
{

namespace chassis
{

void ChassisDriveCommand::initialize()
{}

aruwlib::algorithms::ExtendedKalman imuGyroKalman(1.0f, 0.0f);

float filteredGyroRotation;
float chassisDesiredRotationSpeed;
float error;
float chassisRotationDesiredWheelspeed;

const float MAX_CHASSIS_ROTATION_VELOCITY = 10.0f;

void ChassisDriveCommand::execute()
{
    // rotation in degrees per second
    filteredGyroRotation = imuGyroKalman.filterData(aruwlib::algorithms::degreesToRadians(
        aruwlib::sensors::Mpu6500::getGz() / aruwlib::sensors::Mpu6500::LSB_D_PER_S_TO_D_PER_S));
    chassisDesiredRotationSpeed = -ChassisSubsystem::getChassisR() * MAX_CHASSIS_ROTATION_VELOCITY;
    error = chassisDesiredRotationSpeed - filteredGyroRotation;

    if (aruwlib::Remote::getSwitch(aruwlib::Remote::Switch::LEFT_SWITCH) == aruwlib::Remote::SwitchState::UP)
    {
        chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(error, -700.0f);
    }
    else
    {
        chassisRotationDesiredWheelspeed = ChassisSubsystem::getChassisR()
            * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    }

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

void ChassisDriveCommand::end(bool interrupted)
{
    if (interrupted)
    {
        chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
    }
    chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
}

bool ChassisDriveCommand::isFinished() const
{
    return false;
}

}  // namespace chassis

}  // namespace aruwsrc
