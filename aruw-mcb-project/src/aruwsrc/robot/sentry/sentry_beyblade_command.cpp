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

#include "sentry_beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/robot/sentry/sentry_chassis_rel_drive.hpp"

using namespace tap::algorithms;

namespace aruwsrc
{
namespace sentry
{
SentryBeybladeCommand::SentryBeybladeCommand(
    tap::Drivers* drivers,
    aruwsrc::chassis::HolonomicChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    aruwsrc::control::sentry::SentryControlOperatorInterface& operatorInterface,
    const tap::algorithms::transforms::Transform& worldToChassis,
    const SentryBeybladeConfig config)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      operatorInterface(operatorInterface),
      worldToChassis(worldToChassis),
      config(config)
{
    addSubsystemRequirement(chassis);
}

// Resets ramp
void SentryBeybladeCommand::initialize()
{
#ifdef ENV_UNIT_TESTS
    rotationDirection = 1;
#else
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    rotateSpeedRamp.reset(chassis->getDesiredRotation());
}

void SentryBeybladeCommand::execute()
{
    if (yawMotor->isOnline())
    {
        // Gets current turret yaw angle
        // float turretYawAngle = yawMotor->getAngleFromCenter();

        float worldYawAngle = -worldToChassis.getYaw();

        float x = 0.0f;
        float y = 0.0f;
        // Note: pass in 0 as rotation since we don't want to take into consideration
        // scaling due to rotation as this will be fairly constant and thus it isn't
        // worth scaling here.
        sentry::SentryChassisRelDrive::computeDesiredUserTranslation(
            &operatorInterface,
            drivers,
            chassis,
            0,
            &x,
            &y);
        x *= config.beybladeTranslationalSpeedMultiplier;
        y *= config.beybladeTranslationalSpeedMultiplier;

        const float maxWheelSpeed = aruwsrc::chassis::HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled up
        // by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed)
        const float translationalSpeedThreshold =
            config.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
            config.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

        float rampTarget =
            rotationDirection * config.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

        // reduce the beyblade rotation when translating to allow for better translational speed
        // (otherwise it is likely that you will barely move unless
        // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
        if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold)
        {
            rampTarget *= config.beybladeRotationalSpeedMultiplierWhenTranslating;
        }

        rotateSpeedRamp.setTarget(rampTarget);
        // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
        rotateSpeedRamp.update(config.beybladeRampRate);
        float r = rotateSpeedRamp.getValue();

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, worldYawAngle);

        // set outputs
        chassis->setDesiredOutput(x, y, r);
    }
    else
    {
        SentryChassisRelDrive::onExecute(&operatorInterface, drivers, chassis);
    }
}

void SentryBeybladeCommand::end(bool) { chassis->setZeroRPM(); }
}  // namespace sentry

}  // namespace aruwsrc
