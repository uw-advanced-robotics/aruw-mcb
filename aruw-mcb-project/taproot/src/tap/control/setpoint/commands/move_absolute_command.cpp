/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "move_absolute_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/clock.hpp"

namespace tap
{
namespace control
{
namespace setpoint
{
MoveAbsoluteCommand::MoveAbsoluteCommand(
    SetpointSubsystem* setpointSubsystem,
    float setpoint,
    float speed,
    float setpointTolerance,
    bool automaticallyClearJam,
    bool setSetpointToTargetOnEnd)
    : setpointSubsystem(setpointSubsystem),
      setpoint(setpoint),
      speed(speed),
      setpointTolerance(setpointTolerance),
      automaticallyClearJam(automaticallyClearJam),
      setSetpointToTargetOnEnd(setSetpointToTargetOnEnd)
{
    this->addSubsystemRequirement(setpointSubsystem);
}

bool MoveAbsoluteCommand::isReady()
{
    return !setpointSubsystem->isJammed() && setpointSubsystem->isOnline();
}

void MoveAbsoluteCommand::initialize()
{
    rampToSetpoint.setTarget(setpoint);
    rampToSetpoint.setValue(setpointSubsystem->getCurrentValue());
    prevMoveTime = tap::arch::clock::getTimeMilliseconds();
}

void MoveAbsoluteCommand::execute()
{
    // If the subsystem is jammed, set the setpoint to the current value. Necessary since
    // derived classes may choose to overwrite the `isFinished` function and so for motor safety
    // we do this. Also if subsystem is offline we delay our execution so that setpoint doesn't
    // run away while subsystem offline.
    if (setpointSubsystem->isJammed() || !setpointSubsystem->isOnline())
    {
        // Set prevMoveTime to now so that delta time doesn't become ridiculous while
        // subsystem is stuck in non-functional state.
        prevMoveTime = tap::arch::clock::getTimeMilliseconds();
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
        return;
    }

    // We can assume that subsystem is connected, otherwise end will be called.
    uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
    // Divide by 1'000 to get setpoint-units because speed is in setpoint-units/second
    // and time interval is in milliseconds. (milliseconds * 1/1000 (second/millisecond) *
    // (setpoint-units/second))  = 1/1000 setpoint-units as our conversion
    rampToSetpoint.update((static_cast<float>(currTime - prevMoveTime) * speed) / 1000.0f);
    prevMoveTime = currTime;

    setpointSubsystem->setSetpoint(rampToSetpoint.getValue());
}

void MoveAbsoluteCommand::end(bool)
{
    // Either set setpoint to ideal target or current value based on option
    // used to construct command.
    if (!setpointSubsystem->isJammed() && setSetpointToTargetOnEnd)
    {
        setpointSubsystem->setSetpoint(setpoint);
    }
    else
    {
        setpointSubsystem->setSetpoint(setpointSubsystem->getCurrentValue());
    }

    if (automaticallyClearJam)
    {
        setpointSubsystem->clearJam();
    }
}

bool MoveAbsoluteCommand::isFinished() const
{
    // Command is finished if we've reached target or if our subsystem is jammed
    // or offline
    return (rampToSetpoint.isTargetReached() && algorithms::compareFloatClose(
                                                    setpointSubsystem->getCurrentValue(),
                                                    rampToSetpoint.getTarget(),
                                                    setpointTolerance)) ||
           setpointSubsystem->isJammed() || !setpointSubsystem->isOnline();
}

}  // namespace setpoint

}  // namespace control

}  // namespace tap
