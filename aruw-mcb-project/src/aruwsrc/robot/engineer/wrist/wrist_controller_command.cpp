/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "aruwsrc/robot/engineer/wrist/wrist_controller_command.hpp"

#include "aruwsrc/robot/engineer/engineer_wrist_constants.hpp"

namespace aruwsrc::engineer::wrist
{
WristControllerCommand::WristControllerCommand(
    WristSubsystem &wrist,
    EngineerControlOperatorInterface *operatorInterface,
    std::optional<std::pair<uint8_t, float>> axisAndSetpoint,
    std::optional<uint8_t *> commandCounter,
    std::optional<bool *> isSettled)
    : tap::control::Command(),
      wrist(wrist),
      operatorInterface(operatorInterface),
      axisAndSetpoint(axisAndSetpoint),
      commandCounter(commandCounter),
      isSettled(isSettled)
{
    addSubsystemRequirement(&wrist);
}

void WristControllerCommand::initialize()
{
    setpointHoldTimer.stop();
    maxTimeoutTimer.restart(MAX_COMMAND_TIMEOUT);
    bool axesToSet[3] = {true, true, true};
    if (axisAndSetpoint.has_value())
    {
        // keep the other two axes static and only set the one specified
        const auto [axis, setpoint] = axisAndSetpoint.value();
        switch (axis)
        {
            case 0:
                wrist.setSetpointTheta1(setpoint);
                axesToSet[0] = false;
                break;
            case 1:
                wrist.setSetpointTheta2(setpoint);
                axesToSet[1] = false;
                break;
            case 2:
                wrist.setSetpointTheta3(setpoint);
                axesToSet[2] = false;
                break;
            default:
                break;
        }
    }

    // set the axes that are not being tuned to their current setpoints
    if (axesToSet[0]) wrist.setSetpointTheta1(wrist.getTheta1());
    if (axesToSet[1]) wrist.setSetpointTheta2(wrist.getTheta2());
    if (axesToSet[2]) wrist.setSetpointTheta3(wrist.getTheta3());
}

void WristControllerCommand::execute()
{
    if (isSettled.has_value())
    {
        **isSettled = selectedAxisIsAtSetpoint() && !wrist.isWristMoving();
    }
    // control from operator interface when not tuning a specific axis
    if (axisAndSetpoint.has_value())
    {
        if (selectedAxisIsAtSetpoint())
        {
            if (setpointHoldTimer.isStopped())
            {
                setpointHoldTimer.restart(SETPOINT_HOLD_TIME_MS);
            }
        }
        else
        {
            // reset timer if not within setpoint epsilon
            setpointHoldTimer.stop();
        }
    }
    else
    {
        // Get the desired velocities from the operator interface
        // to add/subtract from position setpoint
        float theta1Velocity =
            operatorInterface->getWristTheta1Velocity() * WRIST_THETA_1_SCALING_FACTOR;
        float theta2Velocity =
            operatorInterface->getWristTheta2Velocity() * WRIST_THETA_2_SCALING_FACTOR;
        float theta3Velocity =
            operatorInterface->getWristTheta3Velocity() * WRIST_THETA_3_SCALING_FACTOR;

        // Set the desired positions
        wrist.setSetpointTheta3(wrist.getSetpointTheta3() + theta3Velocity);  // theta3 is "roll"
        wrist.setSetpointTheta2(wrist.getSetpointTheta2() + theta2Velocity);  // theta2 is "pitch"
        wrist.setSetpointTheta1(wrist.getSetpointTheta1() + theta1Velocity);  // theta1 is "yaw"
    }
}

bool WristControllerCommand::isFinished() const
{
    // only finishes in tuning mode once setpoint has been held for 2 seconds or max timeout has
    // been reached
    return (axisAndSetpoint.has_value() && setpointHoldTimer.isExpired() &&
            !setpointHoldTimer.isStopped()) ||
           (axisAndSetpoint.has_value() && maxTimeoutTimer.isExpired());
}

bool WristControllerCommand::selectedAxisIsAtSetpoint() const
{
    if (!axisAndSetpoint.has_value()) return false;

    switch (axisAndSetpoint->first)
    {
        case 0:
            return wrist.atSetpointTheta1(SETPOINT_EPSILON);
        case 1:
            return wrist.atSetpointTheta2(SETPOINT_EPSILON);
        case 2:
            return wrist.atSetpointTheta3(SETPOINT_EPSILON);
        default:
            return true;
    }
}

void WristControllerCommand::end(bool)
{
    if (commandCounter.has_value())
    {
        (**commandCounter)++;
    }
}

}  // namespace aruwsrc::engineer::wrist