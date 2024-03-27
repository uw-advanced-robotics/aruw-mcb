/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_subsystem.hpp"

#include "chassis_rel_drive.hpp"
#include "holonomic_chassis_subsystem.hpp"

using namespace tap::algorithms;
using namespace aruwsrc::control::turret;

namespace aruwsrc::chassis
{
ChassisAutorotateCommand::ChassisAutorotateCommand(
    tap::Drivers* drivers,
    aruwsrc::control::ControlOperatorInterface* operatorInterface,
    HolonomicChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    ChassisSymmetry chassisSymmetry)
    : drivers(drivers),
      operatorInterface(operatorInterface),
      chassis(chassis),
      yawMotor(yawMotor),
      chassisSymmetry(chassisSymmetry),
      chassisAutorotating(true)
{
    addSubsystemRequirement(chassis);
}

void ChassisAutorotateCommand::initialize()
{
    desiredRotationAverage = chassis->getDesiredRotation();
}

void ChassisAutorotateCommand::updateAutorotateState()
{
    float turretYawActualSetpointDiff = abs(yawMotor->getValidChassisMeasurementError());

    if (chassisAutorotating && chassisSymmetry != ChassisSymmetry::SYMMETRICAL_NONE &&
        !yawMotor->getConfig().limitMotorAngles &&
        turretYawActualSetpointDiff > (M_PI - TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION))
    {
        // If turret setpoint all of a sudden turns around, don't autorotate
        chassisAutorotating = false;
    }
    else if (
        !chassisAutorotating &&
        turretYawActualSetpointDiff < TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION)
    {
        // Once the turret setpoint/target have reached each other, start turning again
        chassisAutorotating = true;
    }
}

float ChassisAutorotateCommand::computeAngleFromCenterForAutorotation(
    float turretAngleFromCenter,
    float maxAngleFromCenter)
{
    return WrappedFloat(turretAngleFromCenter, -maxAngleFromCenter, maxAngleFromCenter)
        .getWrappedValue();
}

void ChassisAutorotateCommand::execute()
{
    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (yawMotor->isOnline())
    {
        updateAutorotateState();

        float turretAngleFromCenter = yawMotor->getAngleFromCenter();

        if (chassisAutorotating)
        {
            float maxAngleFromCenter = M_PI;

            if (!yawMotor->getConfig().limitMotorAngles)
            {
                switch (chassisSymmetry)
                {
                    case ChassisSymmetry::SYMMETRICAL_180:
                        maxAngleFromCenter = M_PI_2;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_90:
                        maxAngleFromCenter = M_PI_4;
                        break;
                    case ChassisSymmetry::SYMMETRICAL_NONE:
                    default:
                        break;
                }
            }
            float angleFromCenterForChassisAutorotate =
                computeAngleFromCenterForAutorotation(turretAngleFromCenter, maxAngleFromCenter);
            // PD controller to find desired rotational component of the chassis control
            float desiredRotation = chassis->chassisSpeedRotationPID(
                angleFromCenterForChassisAutorotate,
                yawMotor->getChassisFrameVelocity() - modm::toRadian(drivers->mpu6500.getGz()));

            // find an alpha value to be used for the low pass filter, some value >
            // AUTOROTATION_MIN_SMOOTHING_ALPHA, inversely proportional to
            // angleFromCenterForChassisAutorotate, so when autorotate angle error is large, low
            // pass filter alpha is small and more averaging will be applied to the desired
            // autorotation
            float autorotateSmoothingAlpha = std::max(
                1.0f - abs(angleFromCenterForChassisAutorotate) / maxAngleFromCenter,
                AUTOROTATION_MIN_SMOOTHING_ALPHA);

            // low pass filter the desiredRotation to avoid radical changes in the desired
            // rotation when far away from where we are centering the chassis around
            desiredRotationAverage =
                lowPassFilter(desiredRotationAverage, desiredRotation, autorotateSmoothingAlpha);
        }

        const float maxWheelSpeed = HolonomicChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // the x/y translational speed is limited to this value, this means when rotation is
        // large, the translational speed will be clamped to a smaller value to compensate
        float rotationLimitedMaxTranslationalSpeed =
            maxWheelSpeed * chassis->calculateRotationTranslationalGain(desiredRotationAverage);

        float chassisXDesiredWheelspeed = limitVal(
            operatorInterface->getChassisXInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        float chassisYDesiredWheelspeed = limitVal(
            operatorInterface->getChassisYInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        // Rotate X and Y depending on turret angle
        rotateVector(&chassisXDesiredWheelspeed, &chassisYDesiredWheelspeed, turretAngleFromCenter);

        chassis->setDesiredOutput(
            chassisXDesiredWheelspeed,
            chassisYDesiredWheelspeed,
            desiredRotationAverage);
    }
    else
    {
        ChassisRelDrive::onExecute(operatorInterface, drivers, chassis);
    }
}

void ChassisAutorotateCommand::end(bool) { chassis->setZeroRPM(); }

bool ChassisAutorotateCommand::isFinished() const { return false; }

}  // namespace aruwsrc::chassis
