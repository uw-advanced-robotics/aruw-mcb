/*
 * Copyright (c) 2020-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"  // @todo I have a whole issue with this but this will have to do for now, lest we go through more massive refactors than we have to
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/control/turret/turret_subsystem.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc
{
namespace chassis
{
BeybladeCommand::BeybladeCommand(
    tap::Drivers* drivers,
    HolonomicChassisSubsystem& chassis,
    const aruwsrc::control::turret::TurretMotor& yawMotor,
    aruwsrc::control::ControlOperatorInterface& operatorInterface)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&chassis);
}

// Resets ramp
void BeybladeCommand::initialize()
{
#ifdef ENV_UNIT_TESTS
    rotationDirection = 1;
#else
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;
#endif
    rotateSpeedRamp.reset(chassis.getDesiredRotation());
}

void BeybladeCommand::execute()
{
    if (yawMotor.isOnline())
    {
        // Gets current turret yaw angle
        float turretYawAngle = yawMotor.getAngleFromCenter();

        float x = 0.0f;
        float y = 0.0f;
        // Note: pass in 0 as rotation since we don't want to take into consideration
        // scaling due to rotation as this will be fairly constant and thus it isn't
        // worth scaling here.
        ChassisRelDrive::computeDesiredUserTranslation(
            &operatorInterface,
            drivers,
            &chassis,
            0,
            &x,
            &y);
        x *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;
        y *= BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER;

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled up
        // by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed)
        const float translationalSpeedThreshold =
            BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE *
            BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER * maxWheelSpeed;

        float rampTarget =
            rotationDirection * BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX * maxWheelSpeed;

        // reduce the beyblade rotation when translating to allow for better translational speed
        // (otherwise it is likely that you will barely move unless
        // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
        if (fabsf(x) > translationalSpeedThreshold || fabsf(y) > translationalSpeedThreshold)
        {
            rampTarget *= BEYBLADE_ROTATIONAL_SPEED_MULTIPLIER_WHEN_TRANSLATING;
        }

        rotateSpeedRamp.setTarget(rampTarget);
        // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
        rotateSpeedRamp.update(BEYBLADE_RAMP_UPDATE_RAMP);
        float r = rotateSpeedRamp.getValue();

        // Rotate X and Y depending on turret angle
        tap::algorithms::rotateVector(&x, &y, turretYawAngle);

        // set outputs
        chassis.setDesiredOutput(x, y, r);
    }
    else
    {
        ChassisRelDrive::onExecute(&operatorInterface, drivers, &chassis);
    }
}

void BeybladeCommand::end(bool) { chassis.setZeroRPM(); }
}  // namespace chassis

}  // namespace aruwsrc
