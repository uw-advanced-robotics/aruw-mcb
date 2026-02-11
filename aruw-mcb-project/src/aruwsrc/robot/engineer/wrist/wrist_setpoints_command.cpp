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

#include "aruwsrc/robot/engineer/wrist/wrist_setpoints_command.hpp"
namespace aruwsrc::engineer::wrist
{
WristSetpointsCommand::WristSetpointsCommand(WristSubsystem &wrist, std::vector<Setpoint> setpoints)
    : wrist(wrist),
      setpoints(std::move(setpoints)),
      currentSetpointIndex(0)
{
    addSubsystemRequirement(&wrist);
}

void WristSetpointsCommand::initialize() {}

void WristSetpointsCommand::execute()
{
    if (currentSetpointIndex < setpoints.size())
    {
        const auto &setpoint = setpoints[currentSetpointIndex];
        wrist.setSetpointTheta2(setpoint.pitch);  // theta2 is pitch
        wrist.setSetpointTheta1(setpoint.yaw);    // theta1 is yaw
        if (wrist.atSetpointTheta2(setpoint.epsilonPitch) &&
            wrist.atSetpointTheta1(setpoint.epsilonYaw))
        {
            currentSetpointIndex++;
        }
    }
}

void WristSetpointsCommand::end(bool) {}

bool WristSetpointsCommand::isFinished() const { return currentSetpointIndex >= setpoints.size(); }
}  // namespace aruwsrc::engineer::wrist
