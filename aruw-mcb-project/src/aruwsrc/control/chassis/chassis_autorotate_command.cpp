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

#include "chassis_autorotate_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/serial/remote.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"
#include "aruwsrc/drivers.hpp"

#include "chassis_rel_drive.hpp"
#include "chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::control::turret;

namespace aruwsrc
{
namespace chassis
{
ChassisAutorotateCommand::ChassisAutorotateCommand(
    aruwsrc::Drivers* drivers,
    ChassisSubsystem* chassis,
    const tap::control::turret::TurretSubsystemInterface* turret,
    ChassisSymmetry chassisSymmetry)
    : drivers(drivers),
      chassis(chassis),
      turret(turret),
      chassisSymmetry(chassisSymmetry),
      chassisAutorotating(true)
{
    addSubsystemRequirement(chassis);
}

void ChassisAutorotateCommand::initialize()
{
    desiredRotationAverage = chassis->getDesiredRotation();
}

void ChassisAutorotateCommand::updateAutorotateState(
    const tap::control::turret::TurretSubsystemInterface* turret)
{
    float turretYawActualSetpointDiff =
        abs(turret->getCurrentYawValue().difference(turret->getYawSetpoint()));

    if (chassisAutorotating && chassisSymmetry != ChassisSymmetry::SYMMETRICAL_NONE &&
        !turret->yawLimited() &&
        turretYawActualSetpointDiff > (180 - SETPOINT_AND_CURRENT_YAW_MATCH_THRESHOLD))
    {
        // If turret setpoint all of a sudden turns around, don't autorotate
        chassisAutorotating = false;
    }
    else if (
        !chassisAutorotating &&
        turretYawActualSetpointDiff < SETPOINT_AND_CURRENT_YAW_MATCH_THRESHOLD)
    {
        // Once the turret setpoint/target have reached each other, start turning again
        chassisAutorotating = true;
    }
}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (turret->isOnline())
    {
        updateAutorotateState(turret);

        float turretAngleFromCenter = turret->getYawAngleFromCenter();

        if (chassisAutorotating)
        {
            float maxAngleFromCenter = 180.0f;

            if (!turret->yawLimited())
            {
                switch (chassisSymmetry)
                {
                    case ChassisSymmetry::SYMMETRICAL_180:
                        maxAngleFromCenter = 90.0f;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_90:
                        maxAngleFromCenter = 45.0f;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_NONE:
                    default:
                        break;
                }
            }

            float angleFromCenterForChassisAutorotate =
                ContiguousFloat(turretAngleFromCenter, -maxAngleFromCenter, maxAngleFromCenter)
                    .getValue();

            // PD controller to find desired rotational component of the chassis control
            float desiredRotation = chassis->chassisSpeedRotationPID(
                angleFromCenterForChassisAutorotate,
                turret->getYawVelocity() - drivers->mpu6500.getGz());

            // find an alpha value to be used for the low pass filter, some value >
            // AUTOROTATE_MIN_SMOOTHING_ALPHA, inversely proportional to
            // angleFromCenterForChassisAutorotate, so when autorotate angle error is large, low
            // pass filter alpha is small and more averaging will be applied to the desired
            // autorotation
            float autorotateSmoothingAlpha = std::max(
                1.0f - abs(angleFromCenterForChassisAutorotate) / maxAngleFromCenter,
                AUTOROTATE_MIN_SMOOTHING_ALPHA);

            // low pass filter the desiredRotation
            desiredRotationAverage =
                lowPassFilter(desiredRotationAverage, desiredRotation, autorotateSmoothingAlpha);
        }

        const float MAX_WHEEL_SPEED = ChassisSubsystem::getMaxUserWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // what we will multiply x and y speed by to take into account rotation
        float rTranslationalGain =
            MAX_WHEEL_SPEED * chassis->calculateRotationTranslationalGain(desiredRotationAverage);

        float chassisXDesiredWheelspeed = limitVal(
            drivers->controlOperatorInterface.getChassisXInput(),
            -rTranslationalGain,
            rTranslationalGain);

        float chassisYDesiredWheelspeed = limitVal(
            drivers->controlOperatorInterface.getChassisYInput(),
            -rTranslationalGain,
            rTranslationalGain);
        // Rotate X and Y depending on turret angle
        rotateVector(
            &chassisXDesiredWheelspeed,
            &chassisYDesiredWheelspeed,
            modm::toRadian(turretAngleFromCenter));

        chassis->setDesiredOutput(
            chassisXDesiredWheelspeed,
            chassisYDesiredWheelspeed,
            desiredRotationAverage);
    }
    else
    {
        ChassisRelDrive::onExecute(drivers, chassis);
    }
}

void ChassisAutorotateCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisAutorotateCommand::isFinished() const { return false; }

}  // namespace chassis

}  // namespace aruwsrc
