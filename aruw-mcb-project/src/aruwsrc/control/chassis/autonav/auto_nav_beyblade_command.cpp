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
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

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
    aruwsrc::communication::serial::SentryResponseTransmitter& sentryResponseTransmitter,
    const aruwsrc::sentry::SentryBeybladeCommand::SentryBeybladeConfig config,
    bool beybladeOnlyInGame)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      visionCoprocessor(visionCoprocessor),
      odometryInterface(odometryInterface),
      sentryResponseTransmitter(sentryResponseTransmitter),
      config(config),
      beybladeOnlyInGame(beybladeOnlyInGame)
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

    sentryResponseTransmitter.queueRequest(
            beybladeEnabled ? aruwsrc::communication::serial::SentryResponseType::BEYBLADE_ENABLED : aruwsrc::communication::serial::SentryResponseType::BEYBLADE_DISABLED);
    
    sentryResponseTransmitter.queueRequest(
            movementEnabled ? aruwsrc::communication::serial::SentryResponseType::MOVEMENT_ENABLED : aruwsrc::communication::serial::SentryResponseType::MOVEMENT_DISABLED);
}

void AutoNavBeybladeCommand::execute()
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

        float x = 0.0;
        float y = 0.0;
        float rampTarget = 0.0;

        aruwsrc::serial::VisionCoprocessor::AutoNavSetpointData setpointData =
            visionCoprocessor.getLastSetpointData();
        
        if (setpointData.pathFound && visionCoprocessor.isCvOnline())
        {
            float desiredVelocityX = setpointData.x - currentX;
            float desiredVelocityY = setpointData.y - currentY;
            float mag = sqrtf(pow(desiredVelocityX, 2) + pow(desiredVelocityY, 2));
            if (mag > 0.01)
            {
                x = desiredVelocityX / mag * config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;
                y = desiredVelocityY / mag * config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;
            }
        }

        if ((!beybladeOnlyInGame || drivers.refSerial.getGameData().gameStage == RefSerial::Rx::GameStage::IN_GAME) && beybladeEnabled)
        {
            // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled up
            // by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed)
            const float translationalSpeedThreshold =
                config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
                config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

            rampTarget = rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

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
