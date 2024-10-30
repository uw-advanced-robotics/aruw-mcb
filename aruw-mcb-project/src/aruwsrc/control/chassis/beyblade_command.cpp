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

#include "beyblade_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_rel_drive.hpp"
#include "holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::arch::clock;

namespace aruwsrc
{
namespace chassis
{
BeybladeCommand::BeybladeCommand(
    tap::Drivers* drivers,
    HolonomicChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    aruwsrc::control::ControlOperatorInterface& operatorInterface,
    float rotationMultiplier)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      operatorInterface(operatorInterface),
      rotationMultiplier(rotationMultiplier),
      lastHitTime(getTimeMilliseconds())
{
    addSubsystemRequirement(chassis);
}

// Resets ramp
void BeybladeCommand::initialize()
{
#ifdef ENV_UNIT_TESTS
    rotationDirection = 1;
#else
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    rotateSpeedRamp.reset(chassis->getDesiredRotation());
}

void BeybladeCommand::execute()
{
    if (yawMotor->isOnline())
    {
        // Gets current turret yaw angle
        float turretYawAngle = yawMotor->getAngleFromCenter();

        float x = 0.0f;
        float y = 0.0f;
        // Note: pass in 0 as rotation since we don't want to take into consideration
        // scaling due to rotation as this will be fairly constant and thus it isn't
        // worth scaling here.
        ChassisRelDrive::computeDesiredUserTranslation(
            &operatorInterface,
            drivers,
            chassis,
            0,
            &x,
            &y);

        const auto &robotData = this->drivers->refSerial.getRobotData();
        const auto currentTime = getTimeMilliseconds();

        if (robotData.receivedDps > 0) {
            lastHitTime = currentTime;
        }

        const bool inCombat = currentTime - lastHitTime < 10000;

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            HolonomicChassisSubsystem::getChassisPowerLimit(drivers));
        
        // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled up
        // by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed)
        const float translationalSpeedThreshold =
            BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE *
            BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;

        float rampTarget =
            rotationDirection * BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX * maxWheelSpeed;

        if (inCombat) {
            x *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;
            y *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;
        } else {
            x *= 0.9f;
            y *= 0.9f;
        }

        // reduce the beyblade rotation when translating to allow for better translational speed
        // (otherwise it is likely that you will barely move unless
        // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
        if (inCombat && (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold))
        {
            rampTarget *= BEYBLADE_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        } else if (!inCombat) {
            const float f1 = rotationMultiplier * (1.0f - fabsf(x) / maxWheelSpeed);
            const float f2 = rotationMultiplier * (1.0f - fabsf(y) / maxWheelSpeed);

            rampTarget *= std::min(f1, f2);
        }

        rotateSpeedRamp.setTarget(rampTarget);
        // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
        rotateSpeedRamp.update(BEYBLADE_RAMP_UPDATE_RAMP);
        float r = rotateSpeedRamp.getValue();

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, turretYawAngle);

        // set outputs
        chassis->setDesiredOutput(x, y, r);
    }
    else
    {
        ChassisRelDrive::onExecute(&operatorInterface, drivers, chassis);
    }
}

void BeybladeCommand::end(bool) { chassis->setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
