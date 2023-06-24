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

#include "auto_nav_maybe_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::chassis
{
AutoNavMaybeBeybladeCommand::AutoNavMaybeBeybladeCommand(
    tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    const aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface& odometryInterface,
    const aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig config)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      config(config)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

void AutoNavMaybeBeybladeCommand::initialize() {}

void AutoNavMaybeBeybladeCommand::execute()
{
    if (yawMotor.isOnline())
    {
        // Gets current chassis yaw angle
        float currentX = odometryInterface.getCurrentLocation2D().getX();
        float currentY = odometryInterface.getCurrentLocation2D().getY();
        float chassisYawAngle = odometryInterface.getYaw();

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers.refSerial.getRefSerialReceivingData(),
            drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

        // default don't move
        float x = 0.0;
        float y = 0.0;
        float beybladeV = 0.0f;

        if (visionCoprocessor.isCvOnline())
        {

            aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData setpointData =
                visionCoprocessor.getLastSetpointData();

            if (setpointData.pathFound)
            {
                float desiredVelocityX = setpointData.x - currentX;
                float desiredVelocityY = setpointData.y - currentY;
                float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
                if (mag > 0.01)
                {
                    x = desiredVelocityX / mag * maxWheelSpeed;
                    y = desiredVelocityY / mag * maxWheelSpeed;
                }

                x *= SPEED_FACTOR;
                y *= SPEED_FACTOR;
            }

            // @todo incorrect
            beybladeV = beybladeVelocity(maxWheelSpeed, x, y, setpointData);
        } else {
            beybladeV = beybladeVelocity(maxWheelSpeed, x, y);
        }

        // ugly haha!
        if (!movementEnabled) {
            x = 0.0f;
            y = 0.0f;
        }

        if (!beybladeEnabled) {
            beybladeV = 0.0f;
        }

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, -chassisYawAngle); // @todo: this should be correct, but the other robots are incorrect it seems
        // we should debug in ozone to see which implementation is correct, and, if necessary
        // negate the yaw in the sentry kf odometry or negate the yaw in the chassiskf odometry
        // this negation has the potential to mess some things up!

        // set outputs
        chassis.setDesiredOutput(x, y, beybladeV);
    }
}



float AutoNavMaybeBeybladeCommand::beybladeVelocity(
    float maxWheelSpeed,
    float vx,
    float vy,
    aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData setPointData)
{
    if (setPointData.shouldBeyblade){
        const float translationalSpeedThreshold =
            config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
            config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

        float rampTarget =
            ROTATION_DIRECTION * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

        if (fabsf(vx) > translationalSpeedThreshold || fabsf(vy) > translationalSpeedThreshold) {
            rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
        }
        rotateSpeedRamp.setTarget(rampTarget);
        rotateSpeedRamp.update(config.beybladeRampRate);
    } else {
        rotateSpeedRamp.setTarget(0.0f);
        rotateSpeedRamp.update(config.beybladeRampRate);
    }

    return rotateSpeedRamp.getValue();
}

float AutoNavMaybeBeybladeCommand::beybladeVelocity(float maxWheelSpeed, float vx, float vy)
{
    const float translationalSpeedThreshold =
        config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
        config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    float rampTarget =
        ROTATION_DIRECTION * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

    if (fabsf(vx) > translationalSpeedThreshold || fabsf(vy) > translationalSpeedThreshold)
    {
        rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
    }
    rotateSpeedRamp.setTarget(rampTarget);
    rotateSpeedRamp.update(config.beybladeRampRate);

    return rotateSpeedRamp.getValue();
}

void AutoNavMaybeBeybladeCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace aruwsrc::chassis
