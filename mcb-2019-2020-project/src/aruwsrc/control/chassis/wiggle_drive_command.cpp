#include "wiggle_drive_command.hpp"
#include "src/aruwlib/algorithms/contiguous_float.hpp"
#include "src/aruwlib/algorithms/math_user_utils.hpp"
#include "src/aruwlib/communication/remote.hpp"
#include "src/aruwlib/communication/sensors/mpu6500/mpu6500.hpp"

using namespace aruwlib::algorithms;
using namespace aruwlib::sensors;

namespace aruwsrc
{

namespace chassis
{

void WiggleDriveCommand::initialize()
{
    curTime = 0;
}

float WiggleDriveCommand::wiggleSin(uint16_t t)
{
    return 30.0f * sinf((2 * aruwlib::algorithms::PI / 750) * t);
}

void WiggleDriveCommand::execute()
{
    float currentYaw = Mpu6500::getImuAttitude().yaw;
    float turretYawAngle = turret->getYawAngleFromCenter();
    float desiredAngle = wiggleSin(curTime) + (turretYawAngle + currentYaw);  // todo I think this is unnecessary (you add it then subtract it)
    ContiguousFloat rotationError(desiredAngle - currentYaw, -180.0f, 180.0f);
    float r = chassis->chassisSpeedRotationPID(rotationError.getValue(), kP);
    float x = chassis->getChassisX() * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float y = chassis->getChassisY() * ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    float yawOffsetRad = degreesToRadians(turretYawAngle);
    float xRotated = x * cosf(yawOffsetRad) + y * sinf(yawOffsetRad);
    float yRotated = -x * sinf(yawOffsetRad) + y * cosf(yawOffsetRad);
    if (aruwlib::Remote::getSwitch(aruwlib::Remote::Switch::LEFT_SWITCH) ==
            aruwlib::Remote::SwitchState::UP) {
        chassis->setDesiredOutput(0, 0, 0);
    } else {
        chassis->setDesiredOutput(xRotated, yRotated, r);
        curTime++;
    }
}

void WiggleDriveCommand::end(bool interrupted)
{
    if (interrupted)
    {
        chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
    }
    chassis->setDesiredOutput(0.0f, 0.0f, 0.0f);
}

bool WiggleDriveCommand::isFinished() const
{
    return false;
}

}  // namespace chassis

}  // namespace aruwsrc
