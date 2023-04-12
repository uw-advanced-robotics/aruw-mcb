/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "balancing_chassis_autorotate_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

namespace aruwsrc
{
namespace chassis
{
BalancingChassisAutorotateCommand::BalancingChassisAutorotateCommand(
    tap::Drivers* drivers,
    BalancingChassisSubsystem* chassis,
    control::ControlOperatorInterface& operatorInterface,
    const aruwsrc::control::turret::TurretMotor* yawMotor)
    : drivers(drivers),
      chassis(chassis),
      operatorInterface(operatorInterface),
      yawMotor(yawMotor)
{
    assert(chassis != nullptr);
    addSubsystemRequirement(chassis);
}

void BalancingChassisAutorotateCommand::initialize()
{
    chassis->setDesiredOutput(0, 0);
    desiredRotationAverage = 0;
}

void BalancingChassisAutorotateCommand::updateAutorotateState()
{
    float turretYawActualSetpointDiff = abs(yawMotor->getValidChassisMeasurementError());

    if (chassisAutorotating && !yawMotor->getConfig().limitMotorAngles &&
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

void BalancingChassisAutorotateCommand::execute()
{
    uint32_t time = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = time - prevTime;
    prevTime = time;
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
                maxAngleFromCenter = M_PI_2;
            }
            float angleFromCenterForChassisAutorotate = tap::algorithms::ContiguousFloat(
                                                            turretAngleFromCenter,
                                                            -maxAngleFromCenter,
                                                            maxAngleFromCenter)
                                                            .getValue();
            // PD controller to find desired rotational component of the chassis control
            float desiredRotation = chassis->rotationPid.runController(
                angleFromCenterForChassisAutorotate,
                yawMotor->getChassisFrameVelocity() - modm::toRadian(drivers->mpu6500.getGz()),
                dt);

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
            desiredRotationAverage = tap::algorithms::lowPassFilter(
                desiredRotationAverage,
                desiredRotation,
                autorotateSmoothingAlpha);
        }
        float chassisXoutput = 0;
        float chassisRoutput = 0;

        chassis->setDesiredOutput(chassisXoutput, chassisRoutput);
    }
    else
    {
    }
}

void plotPath() {}

void BalancingChassisAutorotateCommand::end(bool interrupted) { chassis->setDesiredOutput(0, 0); }

bool BalancingChassisAutorotateCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc