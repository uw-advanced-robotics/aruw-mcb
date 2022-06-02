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

#include "chassis_diagonal_drive_command.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/chassis/chassis_subsystem.hpp"
#include "aruwsrc/control/chassis/constants/chassis_constants.hpp"
#include "aruwsrc/drivers.hpp"

namespace aruwsrc::chassis
{
ChassisDiagonalDriveCommand::ChassisDiagonalDriveCommand(
    aruwsrc::Drivers* drivers,
    ChassisSubsystem* chassis,
    const aruwsrc::control::turret::TurretMotor* yawMotor,
    ChassisSymmetry chassisSymmetry)
    : ChassisAutorotateCommand(drivers, chassis, yawMotor, chassisSymmetry)
{
    // subsystem requirement added by base class
    assert(chassisSymmetry == ChassisAutorotateCommand::ChassisSymmetry::SYMMETRICAL_90);
}

void ChassisDiagonalDriveCommand::execute()
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
                ContiguousFloat(turretAngleFromCenter, -maxAngleFromCenter, maxAngleFromCenter)
                    .getValue();
            if (!drivers->controlOperatorInterface.isSlowMode())
            {
                if (const auto chassisVelocity = chassis->getActualVelocityChassisRelative();
                    hypot(chassisVelocity[0][0], chassisVelocity[1][0]) >
                    AUTOROTATION_DIAGONAL_SPEED)
                {
                    angleFromCenterForChassisAutorotate =
                        ContiguousFloat(turretAngleFromCenter, -M_PI_2, M_PI_2).getValue() + M_PI_4;
                }
            }

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

            // low pass filter the desiredRotation to avoid radical changes in the desired rotation
            // when far away from where we are centering the chassis around
            desiredRotationAverage =
                lowPassFilter(desiredRotationAverage, desiredRotation, autorotateSmoothingAlpha);
        }

        const float maxWheelSpeed = ChassisSubsystem::getMaxWheelSpeed(
            drivers->refSerial.getRefSerialReceivingData(),
            drivers->refSerial.getRobotData().chassis.powerConsumptionLimit);

        // the x/y translational speed is limited to this value, this means when rotation is large,
        // the translational speed will be clamped to a smaller value to compensate
        float rotationLimitedMaxTranslationalSpeed =
            maxWheelSpeed * chassis->calculateRotationTranslationalGain(desiredRotationAverage);

        float chassisXDesiredWheelspeed = limitVal(
            drivers->controlOperatorInterface.getChassisXInput(),
            -rotationLimitedMaxTranslationalSpeed,
            rotationLimitedMaxTranslationalSpeed);

        float chassisYDesiredWheelspeed = limitVal(
            drivers->controlOperatorInterface.getChassisYInput(),
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
        ChassisRelDrive::onExecute(drivers, chassis);
    }
}

}  // namespace aruwsrc::chassis
