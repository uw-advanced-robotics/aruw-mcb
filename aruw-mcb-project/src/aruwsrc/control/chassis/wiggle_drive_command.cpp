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

#include "wiggle_drive_command.hpp"

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/drivers.hpp"

#include "new-chassis/chassis_subsystem.hpp"

#include "chassis_rel_drive.hpp"

using namespace tap::algorithms;
using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc
{
namespace chassis
{
WiggleDriveCommand::WiggleDriveCommand(
    tap::Drivers* drivers,
    ChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    aruwsrc::control::ControlOperatorInterface& operatorInterface)
    : drivers(drivers),
      chassis(chassis),
      yawMotor(yawMotor),
      operatorInterface(operatorInterface),
      rotationSpeedRamp(0)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(chassis));
}

void WiggleDriveCommand::initialize()
{
    rotationSign = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    // TODO replace with chassis rotation speed
    rotationSpeedRamp.reset(0);
    const WiggleParams& wiggleParams = getWiggleParams();
    rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
}

void WiggleDriveCommand::execute()
{
    // We only wiggle when the turret is online.
    if (yawMotor->isOnline())
    {
        const float turretYawFromCenter = yawMotor->getAngleFromCenter();
        const WiggleParams& wiggleParams = getWiggleParams();

        if (turretYawFromCenter > wiggleParams.turnaroundAngle)
        {
            if (rotationSign < 0)
            {
                rotationSign = -rotationSign;
                rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
            }
        }
        else if (turretYawFromCenter < -wiggleParams.turnaroundAngle)
        {
            if (rotationSign > 0)
            {
                rotationSign = -rotationSign;
                rotationSpeedRamp.setTarget(rotationSign * wiggleParams.rotationSpeed);
            }
        }

        rotationSpeedRamp.update(wiggleParams.rotationSpeedIncrement);

        float r = rotationSpeedRamp.getValue();

        float x = 0.0f;
        float y = 0.0f;
        ChassisRelDrive::computeDesiredUserTranslation(
            &operatorInterface,
            drivers,
            chassis,
            r,
            &x,
            &y);
        x *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        y *= TRANSLATIONAL_SPEED_FRACTION_WHILE_WIGGLING;
        // Apply a rotation matrix to the user input so you drive turret
        // relative while wiggling.
        rotateVector(&x, &y, turretYawFromCenter);

        chassis->setDesiredOutput(x, y, r);
    }
    else
    {
        ChassisRelDrive::onExecute(&operatorInterface, drivers, chassis);
    }
}

void WiggleDriveCommand::end(bool) { chassis->setZeroRPM(); }

bool WiggleDriveCommand::isFinished() const { return false; }

const WiggleDriveCommand::WiggleParams& WiggleDriveCommand::getWiggleParams() const
{
    return WIGGLE_PARAMS_45W_CUTOFF;
    uint16_t powerConsumptionLimit =
        drivers->refSerial.getRobotData().chassis.powerConsumptionLimit;
    if (powerConsumptionLimit <= 45 || !drivers->refSerial.getRefSerialReceivingData())
    {
        return WIGGLE_PARAMS_45W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 60)
    {
        return WIGGLE_PARAMS_60W_CUTOFF;
    }
    else if (powerConsumptionLimit <= 80)
    {
        return WIGGLE_PARAMS_80W_CUTOFF;
    }
    else
    {
        return WIGGLE_PARAMS_MAX_CUTOFF;
    }
}

}  // namespace chassis

}  // namespace aruwsrc
