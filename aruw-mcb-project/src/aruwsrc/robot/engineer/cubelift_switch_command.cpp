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

#include "cubelift_switch_command.hpp"

#include "engineer_cube_lift_constants.hpp"

namespace aruwsrc::engineer
{
CubeliftSwitchCommand::CubeliftSwitchCommand(
    aruwsrc::control::joint::JointSubsystem& cubeLift,
    bool isDirectionUp)
    : cubeLift(cubeLift),
      isDirectionUp(isDirectionUp)
{
    addSubsystemRequirement(&cubeLift);
}

void CubeliftSwitchCommand::initialize()
{
    float setpoint = cubeLift.getSetpoint();
    if (isDirectionUp)
    {
        if (setpoint == TWO_CUBE_SETPOINT)
        {
            cubeLift.setSetpoint(ONE_CUBE_SETPOINT);
        }
        else if (setpoint == THREE_CUBE_SETPOINT)
        {
            cubeLift.setSetpoint(TWO_CUBE_SETPOINT);
        }
    }
    else
    {
        if (setpoint == ONE_CUBE_SETPOINT)
        {
            cubeLift.setSetpoint(TWO_CUBE_SETPOINT);
        }
        else if (setpoint == TWO_CUBE_SETPOINT)
        {
            cubeLift.setSetpoint(THREE_CUBE_SETPOINT);
        }
    }
}

void CubeliftSwitchCommand::execute() {}

void CubeliftSwitchCommand::end(bool) {}

bool CubeliftSwitchCommand::isFinished() const { return true; }
}  // namespace aruwsrc::engineer