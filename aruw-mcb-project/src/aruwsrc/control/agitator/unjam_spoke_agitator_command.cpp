/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "unjam_spoke_agitator_command.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"

using namespace tap::algorithms;
using namespace tap::control::setpoint;

namespace aruwsrc::control::agitator
{
UnjamSpokeAgitatorCommand::UnjamSpokeAgitatorCommand(
    IntegrableSetpointSubsystem& integrableSetpointSubsystem,
    const Config& config)
    : integrableSetpointSubsystem(integrableSetpointSubsystem),
      config(config)
{
    assert(config.targetUnjamIntegralChange > 0);
    assert(config.targetCycleCount > 0);
    assert(config.maxWaitTime > 0);

    // max wait time must be > min time it will take to reach the unjam displacement given the unjam
    // velocity
    assert(
        1000.0f * (this->config.targetUnjamIntegralChange / this->config.unjamSetpoint) <
        this->config.maxWaitTime);

    addSubsystemRequirement(&integrableSetpointSubsystem);

    unjamRotateTimeout.stop();
}

bool UnjamSpokeAgitatorCommand::isReady() { return integrableSetpointSubsystem.isOnline(); }

void UnjamSpokeAgitatorCommand::initialize()
{
    unjamRotateTimeout.restart(config.maxWaitTime);

    positionBeforeUnjam = integrableSetpointSubsystem.getCurrentValueIntegral();

    backwardsCount = 0;

    beginUnjamBackwards();
}

void UnjamSpokeAgitatorCommand::execute()
{
    float curPosition = integrableSetpointSubsystem.getCurrentValueIntegral();

    switch (currUnjamState)
    {
        case UNJAM_BACKWARD:
            if ((curPosition <= positionBeforeUnjam - config.targetUnjamIntegralChange) ||
                unjamRotateTimeout.isExpired())
            {
                beginUnjamForwards();
            }
            break;
        case RETURN_FORWARD:
            if (curPosition >= positionBeforeUnjam)
            {
                currUnjamState = JAM_CLEARED;
            }
            else if (unjamRotateTimeout.isExpired())
            {
                beginUnjamBackwards();
            }
            break;
        case JAM_CLEARED:
            break;
    }
}

void UnjamSpokeAgitatorCommand::end(bool)
{
    if (currUnjamState == JAM_CLEARED)
    {
        integrableSetpointSubsystem.clearJam();
    }
    integrableSetpointSubsystem.setSetpoint(0);
}

bool UnjamSpokeAgitatorCommand::isFinished() const
{
    return !integrableSetpointSubsystem.isOnline() || (currUnjamState == JAM_CLEARED) ||
           (backwardsCount >= config.targetCycleCount + 1);
}

void UnjamSpokeAgitatorCommand::beginUnjamForwards()
{
    unjamRotateTimeout.restart(config.maxWaitTime);
    integrableSetpointSubsystem.setSetpoint(config.unjamSetpoint);
    currUnjamState = RETURN_FORWARD;
}

void UnjamSpokeAgitatorCommand::beginUnjamBackwards()
{
    unjamRotateTimeout.restart(config.maxWaitTime);
    integrableSetpointSubsystem.setSetpoint(-config.unjamSetpoint);
    currUnjamState = UNJAM_BACKWARD;
    backwardsCount += 1;
}

}  // namespace aruwsrc::control::agitator
