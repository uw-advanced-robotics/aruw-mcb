#include "chassis_imu_drive_command.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/drivers.hpp"

#include "chassis_rel_drive.hpp"
#include "chassis_subsystem.hpp"

namespace aruwsrc::chassis
{
ChassisImuDriveCommand::ChassisImuDriveCommand(tap::Drivers* drivers, ChassisSubsystem* chassis)
    : tap::control::Command(),
      drivers(drivers),
      chassis(chassis),
      rotationSetpoint(0, 0, 360)
{
    addSubsystemRequirement(chassis);
}

void ChassisImuDriveCommand::initialize()
{
    if (drivers->mpu6500.initialized())
    {
        rotationSetpoint.setValue(drivers->mpu6500.getYaw());
        imuSetpointInitialized = true;
    }
    else
    {
        imuSetpointInitialized = false;
    }
}

void ChassisImuDriveCommand::execute()
{
    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;
    float chassisRotationDesiredWheelspeed = 0.0f;
    float angleFromDesiredRotation = 0.0f;

    if (drivers->mpu6500.initialized())
    {
        if (!imuSetpointInitialized)
        {
            initialize();
        }
        else
        {
            angleFromDesiredRotation = rotationSetpoint.difference(drivers->mpu6500.getYaw());

            // Update desired yaw angle, bound the setpoint to within some angle of the current mpu
            // angle. This way if the chassis is picked up and rotated, it won't try and spin around
            // to get to the same position that it was at previously.
            if (abs(angleFromDesiredRotation) > MAX_ROTATION_ERR)
            {
                // doesn't have to be in the if statement but this is more computationally intensive
                // compared to just shifting the value, so only do this when you actually have to
                // (which is a very small amount in reality).
                rotationSetpoint.setValue(
                    tap::algorithms::ContiguousFloat::limitValue(
                        rotationSetpoint,
                        drivers->mpu6500.getYaw() - MAX_ROTATION_ERR,
                        drivers->mpu6500.getYaw() + MAX_ROTATION_ERR) -
                    drivers->controlOperatorInterface.getChassisRInput() *
                        USER_INPUT_TO_ANGLE_DELTA_SCALAR);
            }
            else
            {
                rotationSetpoint.shiftValue(
                    -drivers->controlOperatorInterface.getChassisRInput() *
                    USER_INPUT_TO_ANGLE_DELTA_SCALAR);
            }

            // run PID controller to attempt to attain the setpoint
            chassisRotationDesiredWheelspeed =
                chassis->chassisSpeedRotationPID(angleFromDesiredRotation, ROTATION_PID_KP);
        }
    }
    else
    {
        chassisRotationDesiredWheelspeed = drivers->controlOperatorInterface.getChassisRInput() *
                                           ChassisSubsystem::MAX_WHEEL_SPEED_SINGLE_MOTOR;
    }

    ChassisRelDrive::computeDesiredUserTranslation(
        drivers,
        chassis,
        chassisRotationDesiredWheelspeed,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed);

    // rotate X and Y depending on IMU angle so you are driving relative to the rotation you want to
    // drive rather than the actual rotation to avoid rotation error windeup that will result in not
    // driving straight in the long run
    tap::algorithms::rotateVector(
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed,
        modm::toRadian(angleFromDesiredRotation));

    chassis->setDesiredOutput(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}

void ChassisImuDriveCommand::end(bool) { chassis->setDesiredOutput(0, 0, 0); }

bool ChassisImuDriveCommand::isFinished() const { return false; }
}  // namespace aruwsrc::chassis
