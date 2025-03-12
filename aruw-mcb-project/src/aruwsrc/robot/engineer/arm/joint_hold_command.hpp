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

#ifndef JOINT_HOLD_COMMAND_HPP_
#define JOINT_HOLD_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/robot/engineer/arm/joint_subsystem.hpp"

namespace aruwsrc
{
namespace engineer
{
/**
 * A command that moves a joint to a specified position.
 */
class JointHoldCommand : public tap::control::Command
{
public:
    JointHoldCommand(JointSubsystem &joint, float setpoint);

    void initialize() override;

    /**
     * Updates the PID controller and applies the output to the joint subsystem.
     */
    void execute() override{};

    void end(bool) override{};

    const char *getName() const override { return "Engineer Joint Hold Command"; };

private:
    JointSubsystem &joint;
    float setpoint;
};

}  // namespace engineer
}  // namespace aruwsrc

#endif  // JOINT_HOLD_COMMAND_HPP_