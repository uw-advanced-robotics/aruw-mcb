/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "chassis_auto_nav_controller.hpp"

#include "tap/communication/serial/ref_serial_data.hpp"

namespace aruwsrc::chassis
{
void ChassisAutoNavController::initialize()
{
    rotationDirection = (rand() - RAND_MAX / 2) < 0 ? -1 : 1;

    lastSetPoint = worldToChassis.getTranslation();
    rotateSpeedRamp.reset(chassis.getDesiredRotation());
}

void ChassisAutoNavController::runController(
    const float maxWheelSpeed,
    const bool movementEnabled,
    const bool beybladeEnabled)
{
    Position currentPos = worldToChassis.getTranslation();  // works bc transformer always makes z 0
    float lookaheadDist = LOOKAHEAD_DISTANCE;  // redeclared here bc it might be useful to replace
                                               // this constant with a function in the future
    Position setpoint = calculateSetPoint(currentPos, lookaheadDist, movementEnabled);

    Vector moveVector = Vector(0, 0, 0);

    Vector posError = setpoint - currentPos;
    float mag = posError.magnitude();

    float desiredSpeed = visionCoprocessor.getAutonavSpeed();

    if (mag > 0.01)
    {
        moveVector = posError / lookaheadDist * desiredSpeed * chassis.mpsToRpm(1);
    }

    // BEYBLADE_TRANSLATIONAL_SPEED_THRESHOLD_MULTIPLIER_FOR_ROTATION_SPEED_DECREASE, scaled
    // up by the current max speed, (BEYBLADE_TRANSLATIONAL_SPEED_MULTIPLIER *
    // maxWheelSpeed)
    const float translationalSpeedThreshold =
        beybladeConfig.translationalSpeedThresholdMultiplierForRotationSpeedDecrease *
        beybladeConfig.beybladeTranslationalSpeedMultiplier * maxWheelSpeed;

    float rampTarget =
        rotationDirection * beybladeConfig.beybladeRotationalSpeedFractionOfMax * maxWheelSpeed;

    // reduce the beyblade rotation when translating to allow for better translational speed
    // (otherwise it is likely that you will barely move unless
    // BEYBLADE_ROTATIONAL_SPEED_FRACTION_OF_MAX is small)
    if (moveVector.magnitude() > translationalSpeedThreshold)
    {
        rampTarget *= beybladeConfig.beybladeRotationalSpeedMultiplierWhenTranslating;
    }

    rotateSpeedRamp.setTarget(rampTarget);
    // Update the r speed by BEYBLADE_RAMP_UPDATE_RAMP each iteration
    rotateSpeedRamp.update(beybladeConfig.beybladeRampRate);
    float r = rotateSpeedRamp.getValue();

    // convert world frame translation to chassis frame
    Vector chassisFrameMoveVector = worldToChassis.apply(moveVector);

    // set outputs
    chassis.setDesiredOutput(
        chassisFrameMoveVector.x(),
        chassisFrameMoveVector.y(),
        beybladeEnabled ? r : 0);
}

Position ChassisAutoNavController::calculateSetPoint(
    Position current,
    float lookaheadDistance,
    bool movementEnabled)
{
    if (!visionCoprocessor.isCvOnline() || !movementEnabled)
    {
        return lastSetPoint;
    }

    if (path.empty())
    {
        return current;
    }

    if (path.hasChanged())
    {
        path.togglePathChanged();
        pathTransitionTimeout.restart(PATH_TRANSITION_TIME_MILLIS);
    }

    float distOfClosest = path.positionToClosestParameter(current);

    Position lookaheadPos = path.parametertoPosition(distOfClosest + lookaheadDistance);

    if (!pathTransitionTimeout.isExpired())
        return quadraticBezierInterpolation(
            lookaheadPos,
            current,
            lastSetPoint,
            (float)pathTransitionTimeout.timeRemaining() / PATH_TRANSITION_TIME_MILLIS);

    lastSetPoint = lookaheadPos;
    return lookaheadPos;
}

}  // namespace aruwsrc::chassis
