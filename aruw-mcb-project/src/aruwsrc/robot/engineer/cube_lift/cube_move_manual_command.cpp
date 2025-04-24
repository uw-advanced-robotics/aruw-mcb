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

#include "cube_move_manual_command.hpp"
namespace aruwsrc::robot::engineer
{
CubeMoveManualCommand::CubeMoveManualCommand(
    CubeStorageSubsystem& cubeLift,
    aruwsrc::control::ControlOperatorInterface* operatorInterface)
    : cubeLift(cubeLift),
      operatorInterface(operatorInterface)
{
    addSubsystemRequirement(&cubeLift);
}

void CubeMoveManualCommand::initialize() { setpoint = cubeLift.getSetpoint(); }

void CubeMoveManualCommand::execute() {
    setpoint += operatorInterface->getTurretPitchInput(0) * MANUAL_MOVE_SPEED;  // right up joystick
    cubeLift.setSetpoint(setpoint);
}

void CubeMoveManualCommand::end(bool) { cubeLift.moveMotor(0); }

bool CubeMoveManualCommand::isFinished() const
{
    return cubeLift.isLimitSwitched();
}  // TODO: change back to isLimitSwitched
}  // namespace aruwsrc::robot::engineer
