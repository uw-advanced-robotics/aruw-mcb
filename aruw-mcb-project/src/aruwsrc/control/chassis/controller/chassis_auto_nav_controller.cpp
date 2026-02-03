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

using namespace tap::algorithms::transforms;

namespace aruwsrc::control::chassis::controller
{
void ChassisAutoNavController::initialize() { lastSetPoint = worldToChassis.getTranslation(); }

Vector ChassisAutoNavController::runFrameRelativeController(const float maxSpeed)
{
    Position currentPos = worldToChassis.getTranslation();  // works bc transformer always makes z 0
    float lookaheadDist = LOOKAHEAD_DISTANCE;  // redeclared here bc it might be useful to replace
                                               // this constant with a function in the future
    Position setpoint = calculateSetPoint(currentPos, lookaheadDist, true);

    Vector posError = setpoint - currentPos;

    if (capBankSubsystem)
    {
        if (posError.magnitude() > translationalMotionThreshold &&
            capBankSubsystem->getAvailableEnergy() > capbankEnergyThreshold)
        {  // is it translating
            capBankSubsystem->changeSprintMode(
                aruwsrc::communication::can::cap_bank::SprintMode::SPRINT);
        }
        else
        {
            capBankSubsystem->changeSprintMode(
                aruwsrc::communication::can::cap_bank::SprintMode::NO_SPRINT);
        }
    }

    if (posError.magnitude() < POS_ERROR_THRESHOLD)
    {
        return Vector(0, 0, 0);
    }

    return posError / lookaheadDist * desiredSpeed;
}

Position ChassisAutoNavController::calculateSetPoint(
    Position current,
    float lookaheadDistance,
    bool movementEnabled)  // todo: remove this?
{
    if (path == nullptr || !movementEnabled || path->empty())
    {
        return lastSetPoint;
    }

    if (path->hasChanged())
    {
        path->clearPathChanged();
        pathTransitionTimeout.restart(PATH_TRANSITION_TIME_MILLIS);
    }

    float distOfClosest = path->positionToClosestParameter(current);

    Position lookaheadPos = path->parametertoPosition(distOfClosest + lookaheadDistance);

    if (!pathTransitionTimeout.isExpired())
        return aruwsrc::algorithms::quadraticBezierInterpolation(
            lookaheadPos,
            current,
            lastSetPoint,
            (float)pathTransitionTimeout.timeRemaining() / PATH_TRANSITION_TIME_MILLIS);

    lastSetPoint = lookaheadPos;
    return lookaheadPos;
}

}  // namespace aruwsrc::control::chassis::controller
