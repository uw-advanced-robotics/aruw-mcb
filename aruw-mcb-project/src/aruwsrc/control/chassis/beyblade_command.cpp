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
#include "tap/communication/sensors/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "chassis_rel_drive.hpp"
#include "chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::sensors;

namespace aruwsrc
{
namespace chassis
{
BeybladeCommand::BeybladeCommand(
    aruwsrc::Drivers* drivers,
    ChassisSubsystem* chassis,
    const tap::control::turret::TurretSubsystemInterface* turret)
    : drivers(drivers),
      chassis(chassis),
      turret(turret),
      rotationalSpeedLinearInterpolator(
          POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT,
          MODM_ARRAY_SIZE(POWER_LIMIT_W_TO_ROTATION_TARGET_RPM_LUT))
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
    if (turret->isOnline())
    {
        // Gets current turret yaw angle
        float turretYawAngle = turret->getYawAngleFromCenter();

        float x = 0.0f;
        float y = 0.0f;
        // Note: pass in 0 as rotation since we don't want to take into consideration
        // scaling due to rotation as this will be fairly constant and thus it isn't
        // worth scaling here.
        ChassisRelDrive::computeDesiredUserTranslation(drivers, chassis, 0, &x, &y);
        x *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;
        y *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;

        const float MAX_WHEEL_SPEED = ChassisSubsystem::getMaxUserWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        const float TRANSLATION_LIMIT =
            BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_FOR_ROTATION_SPEED_DECREASE *
            BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * MAX_WHEEL_SPEED;

        rampTarget = rotationDirection * getRotationTarget();
        if (fabsf(x) > TRANSLATION_LIMIT || fabsf(y) > TRANSLATION_LIMIT)
        {
            rampTarget *= BEYBLADE_ROTATIONAL_SPEED_CUTOFF_WHEN_TRANSLATING;
        }

        rotateSpeedRamp.setTarget(rampTarget);
        // Update the r speed by BEYBLADE_RAMP_UPDATE_RMP of target (linear for each update)
        rotateSpeedRamp.update(BEYBLADE_RAMP_UPDATE_RMP);
        float r = rotateSpeedRamp.getValue();

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, modm::toRadian(turretYawAngle));

        // set outputs
        chassis->setDesiredOutput(x, y, r);
    }
    else
    {
        ChassisRelDrive::onExecute(drivers, chassis);
    }
}

void BeybladeCommand::end(bool) { chassis->setZeroRPM(); }

bool BeybladeCommand::isFinished() const { return false; }

float BeybladeCommand::getRotationTarget() const
{
    const uint16_t powerConsumptionLimit =
        drivers->refSerial.getRefSerialReceivingData()
            ? drivers->refSerial.getRobotData().chassis.powerConsumptionLimit
            : 0;

    return rotationalSpeedLinearInterpolator.interpolate(powerConsumptionLimit);
}
}  // namespace chassis

}  // namespace aruwsrc
