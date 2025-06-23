/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "wrist_setpoints_command.hpp"
namespace aruwsrc::engineer::wrist
{
WristSetpointsCommand::WristSetpointsCommand(WristSubsystem &wrist, std::vector<Setpoint> setpoints)
    : wrist(wrist),
      setpoints(std::move(setpoints)),
      currentSetpointIndex(0)
{
    addSubsystemRequirement(&wrist);
}

void WristSetpointsCommand::initialize()
{
    rampPitch.reset(wrist.getPitch());
    rampPitch.setTarget(setpoints[0].pitch);

    rampYaw.reset(wrist.getYaw());
    rampYaw.setTarget(setpoints[0].yaw);
}

void WristSetpointsCommand::execute()
{
    if (currentSetpointIndex < setpoints.size())
    {
        Setpoint &setpoint = setpoints[currentSetpointIndex];

        if (!rampPitch.isTargetReached()) rampPitch.update(WRIST_SETPOINTS_COMMAND_RAMP_RATE);
        if (!rampYaw.isTargetReached()) rampYaw.update(WRIST_SETPOINTS_COMMAND_RAMP_RATE);

        wrist.setSetpointPitch(rampPitch.getValue());
        wrist.setSetpointYaw(rampYaw.getValue());

        if (wrist.atSetpointPitch(setpoint.epsilonPitch) &&
            wrist.atSetpointYaw(setpoint.epsilonYaw))
        {
            if (++currentSetpointIndex >= setpoints.size()) return;  // All setpoints processed

            setpoint = setpoints[currentSetpointIndex];
            rampPitch.reset(wrist.getPitch());
            rampPitch.setTarget(setpoint.pitch);

            rampYaw.reset(wrist.getYaw());
            rampYaw.setTarget(setpoint.yaw);
        }
    }
}

void WristSetpointsCommand::end(bool) {}

bool WristSetpointsCommand::isFinished() const { return currentSetpointIndex >= setpoints.size(); }
}  // namespace aruwsrc::engineer::wrist
