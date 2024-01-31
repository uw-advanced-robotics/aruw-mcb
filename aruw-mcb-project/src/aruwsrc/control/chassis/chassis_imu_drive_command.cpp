/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "chassis_imu_drive_command.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/robot/control_operator_interface.hpp"

#include "chassis_rel_drive.hpp"
#include "new-chassis/chassis_subsystem.hpp"

using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::chassis
{
ChassisImuDriveCommand::ChassisImuDriveCommand(
    tap::Drivers* drivers,
    aruwsrc::control::ControlOperatorInterface* operatorInterface,
    ChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor)
    : tap::control::Command(),
      drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis),
      yawMotor(yawMotor),
      rotationSetpoint(0, 0, M_TWOPI)
{
    addSubsystemRequirement(chassis);
}

void ChassisImuDriveCommand::initialize()
{
    imuSetpointInitialized =
        drivers->mpu6500.getImuState() == Mpu6500::ImuState::IMU_CALIBRATED ||
        drivers->mpu6500.getImuState() == Mpu6500::ImuState::IMU_NOT_CALIBRATED;

    if (imuSetpointInitialized)
    {
        const float yaw = modm::toRadian(drivers->mpu6500.getYaw());
        rotationSetpoint.setValue(yaw);
    }

    prevTime = tap::arch::clock::getTimeMicroseconds();
}

void ChassisImuDriveCommand::execute()
{
    float chassisRotationDesiredWheelspeed = 0.0f;
    float angleFromDesiredRotation = 0.0f;

    if (drivers->mpu6500.getImuState() == Mpu6500::ImuState::IMU_CALIBRATED ||
        drivers->mpu6500.getImuState() == Mpu6500::ImuState::IMU_NOT_CALIBRATED)
    {
        if (!imuSetpointInitialized)
        {
            initialize();
        }
        else
        {
            const float yaw = modm::toRadian(drivers->mpu6500.getYaw());
            angleFromDesiredRotation = -rotationSetpoint.difference(yaw);

            // Update desired yaw angle, bound the setpoint to within some angle of the current mpu
            // angle. This way if the chassis is picked up and rotated, it won't try and spin around
            // to get to the same position that it was at previously.
            float chassisRInput =
                operatorInterface->getChassisRInput() * USER_INPUT_TO_ANGLE_DELTA_SCALAR;
            if (abs(angleFromDesiredRotation) > MAX_ROTATION_ERR)
            {
                // doesn't have to be in the if statement but this is more computationally intensive
                // compared to just shifting the value, so only do this when you actually have to
                // (which is a very small amount in reality).
                int status = 0;
                rotationSetpoint.setValue(
                    tap::algorithms::ContiguousFloat::limitValue(
                        rotationSetpoint,
                        yaw - MAX_ROTATION_ERR,
                        yaw + MAX_ROTATION_ERR,
                        &status) +
                    chassisRInput);
            }
            else
            {
                rotationSetpoint.shiftValue(chassisRInput);
            }

            uint32_t currTime = tap::arch::clock::getTimeMicroseconds();
            uint32_t dt = currTime - prevTime;
            prevTime = currTime;

            float targetVelocity = (dt == 0) ? 0.0f : chassisRInput * 1'000'000.0f / dt;

            // compute error again now that user input has been updated
            angleFromDesiredRotation = -rotationSetpoint.difference(yaw);

            // run PID controller to attempt to attain the setpoint
            chassisRotationDesiredWheelspeed = chassis->chassisSpeedRotationPID(
                angleFromDesiredRotation,
                targetVelocity - modm::toRadian(drivers->mpu6500.getGz()));
        }
    }
    else
    {
        imuSetpointInitialized = false;
        chassisRotationDesiredWheelspeed = operatorInterface->getChassisRInput();
    }

    float chassisXDesiredWheelspeed = 0.0f;
    float chassisYDesiredWheelspeed = 0.0f;

    ChassisRelDrive::computeDesiredUserTranslation(
        operatorInterface,
        drivers,
        chassis,
        chassisRotationDesiredWheelspeed,
        &chassisXDesiredWheelspeed,
        &chassisYDesiredWheelspeed);

    if (yawMotor != nullptr && yawMotor->isOnline())
    {
        // rotate X and Y based on turret angle from center so translational motion is relative
        // to the turret
        tap::algorithms::rotateVector(
            &chassisXDesiredWheelspeed,
            &chassisYDesiredWheelspeed,
            yawMotor->getAngleFromCenter());
    }
    else
    {
        // rotate X and Y depending on IMU angle so you are driving relative to the rotation you
        // want (the `rotationSetpoint`) to drive rather than the actual rotation to avoid rotation
        // error windeup that causes the chassis to not drive straight in the long run
        tap::algorithms::rotateVector(
            &chassisXDesiredWheelspeed,
            &chassisYDesiredWheelspeed,
            angleFromDesiredRotation);
    }

    chassis->setDesiredOutput(
        chassisXDesiredWheelspeed,
        chassisYDesiredWheelspeed,
        chassisRotationDesiredWheelspeed);
}

void ChassisImuDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisImuDriveCommand::isFinished() const { return false; }
}  // namespace aruwsrc::chassis
