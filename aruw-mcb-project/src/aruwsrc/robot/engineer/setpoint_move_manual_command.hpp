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
#ifndef CUBE_MOVE_MANUAL_COMMAND_HPP_
#define CUBE_MOVE_MANUAL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"
#include "aruwsrc/robot/engineer/limit_switch_setpoint_interface.hpp"

using namespace aruwsrc::engineer;

namespace aruwsrc::robot::engineer

{
class SetpointMoveManualCommand : public tap::control::Command
{
public:
    SetpointMoveManualCommand(
        LimitSwitchSetpointInterface &cubeLift,
        aruwsrc::control::engineer::EngineerControlOperatorInterface *operatorInterface,
        float moveSpeed);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "Cube Move Manual Command"; }

private:
    LimitSwitchSetpointInterface &cubeLift;
    aruwsrc::control::engineer::EngineerControlOperatorInterface *operatorInterface;
    float setpoint, moveSpeed;

};  // class SetpointMovePositionCommand

}  // namespace aruwsrc::robot::engineer
#endif  // CUBE_MOVE_MANUAL_COMMAND_HPP_