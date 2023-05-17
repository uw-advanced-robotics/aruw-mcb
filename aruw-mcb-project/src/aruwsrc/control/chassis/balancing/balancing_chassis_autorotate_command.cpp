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
    if (!tap::algorithms::compareFloatClose(motionDesiredTurretRelative.getLength(), 0, 1))
    {
        chassisMotionPlanning = true;
        return;
    }
    else
    {
        chassisMotionPlanning = false;
    }
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

        // see mode descriptions for why this makes sense
        if (autorotationMode == KEEP_CHASSIS_ANGLE)
        {
            chassisAutorotating = false;
        }
        else if (
            autorotationMode == STRICT_PLATE_FORWARD || autorotationMode == STRICT_SIDE_FORWARD)
        {
            chassisAutorotating = true;
        }
        else if (!lazyTimeout.isExpired())
        {
            chassisAutorotating = true;
        }
    }
}

void BalancingChassisAutorotateCommand::execute()
{
    if (!chassis->getArmState()) chassis->armChassis();
    uint32_t time = tap::arch::clock::getTimeMilliseconds();
    uint32_t dt = time - prevTime;
    prevTime = time;

    // get user input
    motionDesiredTurretRelative = modm::Vector2f(
        operatorInterface.getChassisXInput() * TRANSLATION_REMOTE_SCALAR,
        operatorInterface.getChassisYInput() * TRANSLATION_REMOTE_SCALAR);

    // calculate pid for chassis rotation
    // returns a chassis rotation speed
    if (yawMotor->isOnline())
    {
        bool moveBackwards = false;
        float turretAngleFromCenter = yawMotor->getAngleFromCenter();
        updateAutorotateState();  // initiate or not autorotation
        float chassisRotationSetpoint = 0;

        if (chassisMotionPlanning)
        {
            tap::algorithms::rotateVector(
                &motionDesiredTurretRelative.x,
                &motionDesiredTurretRelative.y,
                turretAngleFromCenter);
            float angleToTarget =
                atan2(motionDesiredTurretRelative.getY(), motionDesiredTurretRelative.getX());
            if (!tap::algorithms::compareFloatClose(angleToTarget, 0, maxAngleFromCenter))
            {
                angleToTarget = angleToTarget + M_PI;
                moveBackwards = true;
            }
            else
            {
                moveBackwards = false;
            }
            chassisRotationSetpoint = getAutorotationSetpoint(angleToTarget);
            debug2 = turretAngleFromCenter;
        }
        else if (chassisAutorotating)
        {
            chassisRotationSetpoint = getAutorotationSetpoint(turretAngleFromCenter);
        }
        debug1 = chassisRotationSetpoint;
        runRotationController(chassisRotationSetpoint, dt);

        // we are now turning the robot towards the desired direction. Apply motion to chassis
        // accordingly.
        float chassisXoutput = motionDesiredTurretRelative.getLength() * (moveBackwards ? -1 : 1);
        float chassisRoutput = DESIRED_ROTATION_SCALAR * desiredRotationAverage;

        chassis->setDesiredOutput(chassisXoutput, chassisRoutput);
    }
    else
    {
        // fall back to chassis drive if no turret
        chassis->setDesiredOutput(
            operatorInterface.getChassisXInput() * TRANSLATION_REMOTE_SCALAR,
            operatorInterface.getChassisYInput() * ROTATION_REMOTE_SCALAR);
    }
    chassis->setDesiredHeight(
        HEIGHT_REMOTE_SCALAR *
        drivers->remote.getChannel(tap::communication::serial::Remote::Channel::WHEEL));
}

float BalancingChassisAutorotateCommand::getAutorotationSetpoint(float turretAngleFromCenter)
{
    return tap::algorithms::ContiguousFloat(
               turretAngleFromCenter,
               -maxAngleFromCenter,
               maxAngleFromCenter)
        .getValue();
}

void BalancingChassisAutorotateCommand::runRotationController(
    float chassisRotationSetpoint,
    float dt)
{
    // PD controller to find desired rotational component of the chassis control
    float desiredRotation = chassis->rotationPid.runController(
        chassisRotationSetpoint,
        yawMotor->getChassisFrameVelocity() - chassis->getChassisOrientationRates().element[2],
        dt);

    // find an alpha value to be used for the low pass filter, some value >
    // AUTOROTATION_MIN_SMOOTHING_ALPHA, inversely proportional to
    // angleFromCenterForChassisAutorotate, so when autorotate angle error is large, low
    // pass filter alpha is small and more averaging will be applied to the desired
    // autorotation
    float autorotateSmoothingAlpha = std::max(
        1.0f - abs(chassisRotationSetpoint) / maxAngleFromCenter,
        AUTOROTATION_MIN_SMOOTHING_ALPHA);

    // low pass filter the desiredRotation to avoid radical changes in the desired
    // rotation when far away from where we are centering the chassis around
    desiredRotationAverage = tap::algorithms::lowPassFilter(
        desiredRotationAverage,
        desiredRotation,
        autorotateSmoothingAlpha);
}

float BalancingChassisAutorotateCommand::plotPath(float turretAngleFromCenter)
{
    tap::algorithms::rotateVector(
        &motionDesiredTurretRelative.x,
        &motionDesiredTurretRelative.y,
        turretAngleFromCenter);
    // ContiguousFloat makes us find the nearest 180 solution to this, instead of overrotating the
    // robot.
    return tap::algorithms::ContiguousFloat(
               atan2(motionDesiredTurretRelative.getY(), motionDesiredTurretRelative.getX()),
               -maxAngleFromCenter,
               maxAngleFromCenter)
        .getValue();
}

void BalancingChassisAutorotateCommand::end(bool interrupted)
{
    chassis->setDesiredOutput(0, 0);
    chassis->disarmChassis();
}

bool BalancingChassisAutorotateCommand::isFinished() const { return false; }
}  // namespace chassis
}  // namespace aruwsrc