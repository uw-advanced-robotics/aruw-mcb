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

#include "auto_nav_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::serial;

namespace aruwsrc
{
namespace chassis
{
AutoNavBeybladeCommand::AutoNavBeybladeCommand(
    tap::Drivers& drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    const aruwsrc::serial::VisionCoprocessor& visionCoprocessor,
    const tap::algorithms::odometry::Odometry2DInterface& odometryInterface,
    const aruwsrc::sentry::SentryBeybladeConfig config,
    tap::algorithms::SmoothPidConfig pidConfig,
    bool autoNavOnlyInGame)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      config(config),
      autoNavOnlyInGame(autoNavOnlyInGame),
      xPid(pidConfig),
      yPid(pidConfig)
{
    // TODO: sucks that we have to pull the address out of the reference bc everything else uses
    // pointers
    addSubsystemRequirement(&chassis);
}

// Resets ramp
void AutoNavBeybladeCommand::initialize()
{
#ifdef ENV_UNIT_TESTS
    rotationDirection = 1;
#else
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    rotateSpeedRamp.reset(chassis.getDesiredRotation());
    xRamp.reset(odometryInterface.getCurrentLocation2D().getX());
    yRamp.reset(odometryInterface.getCurrentLocation2D().getY());
}

void AutoNavBeybladeCommand::execute()
{
    if (yawMotor.isOnline())
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        prevTime = currTime;
        // Gets current chassis yaw angle
        float currentX = odometryInterface.getCurrentLocation2D().getX();
        float currentY = odometryInterface.getCurrentLocation2D().getY();
        float chassisYawAngle = odometryInterface.getYaw();

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers.refSerial.getRefSerialReceivingData(),
            drivers.refSerial.getRobotData().chassis.powerConsumptionLimit);

        float rampTarget = 0.0;

        aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData setpointData =
            visionCoprocessor.getLastSetpointData();
        const tap::communication::serial::RefSerialData::Rx::GameType& gametype =
            drivers.refSerial.getGameData().gameType;

        float x = 0.0f;
        float y = 0.0f;

        if ((int(gametype) == 0 ||
             (drivers.refSerial.getGameData().gameStage == RefSerial::Rx::GameStage::IN_GAME)) &&
            setpointData.pathFound && visionCoprocessor.isCvOnline() && movementEnabled)
        {
            xRamp.setTarget(setpointData.x);
            yRamp.setTarget(setpointData.y);

            xRamp.update(POS_RAMP_RATE);
            yRamp.update(POS_RAMP_RATE);

            float desiredVelocityX = xRamp.getValue() - currentX;
            float desiredVelocityY = yRamp.getValue() - currentY;
            float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
            if (mag > 0.01)
            {
                x = desiredVelocityX / mag * config.beybladeTranslationalSpeedMultiplier *
                    maxWheelSpeed;
                y = desiredVelocityY / mag * config.beybladeTranslationalSpeedMultiplier *
                    maxWheelSpeed;
            }
        }

        // float x = xPid.runControllerDerivateError(xRamp.getValue() - currentX, dt) *
        // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed; float y =
        // yPid.runControllerDerivateError(yRamp.getValue() - currentY, dt) *
        // config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

        if ((int(gametype) == 0 ||
             (drivers.refSerial.getGameData().gameStage == RefSerial::Rx::GameStage::IN_GAME)) &&
            beybladeEnabled)
        {
            // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled
            // up by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER *
            // maxWheelSpeed)
            const float translationalSpeedThreshold =
                config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
                config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

            rampTarget =
                rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

            // reduce the beyblade rotation when translating to allow for better translational speed
            // (otherwise it is likely that you will barely move unless
            // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
            if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold)
            {
                rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
            }
        }

        rotateSpeedRamp.setTarget(rampTarget);
        // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
        rotateSpeedRamp.update(config.beybladeRampRate);
        float r = rotateSpeedRamp.getValue();

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, -chassisYawAngle);

        // set outputs
        chassis.setDesiredOutput(x, y, r);
    }
}

void AutoNavBeybladeCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
