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
#ifndef SETPOINT_MOVE_MANUAL_COMMAND_HPP_
#define SETPOINT_MOVE_MANUAL_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/robot/engineer/engineer_control_operator_interface.hpp"
#include "aruwsrc/robot/engineer/joint_subsystem.hpp"

namespace aruwsrc::engineer
{
enum SetpointType
{
    CUBE_LIFT,
    GANTRY_LIFT,
    GANTRY_EXTENSION
};
class SetpointMoveManualCommand : public tap::control::Command
{
public:
    SetpointMoveManualCommand(
        JointSubsystem &subsystem,
        aruwsrc::control::engineer::EngineerControlOperatorInterface *operatorInterface,
        float moveSpeed,
        SetpointType setpointType = CUBE_LIFT);

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    const char *getName() const override { return "Setpoint Move Manual Command"; }

private:
    JointSubsystem &subsystem;
    aruwsrc::control::engineer::EngineerControlOperatorInterface *operatorInterface;
    float moveSpeed;
    SetpointType setpointType;

};  // class SetpointMovePositionCommand

}  // namespace aruwsrc::engineer
#endif  // SETPOINT_MOVE_MANUAL_COMMAND_HPP_