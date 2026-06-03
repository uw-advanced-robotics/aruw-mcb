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

#ifndef SERVO_MOVE_POSITION_COMMAND_HPP_
#define SERVO_MOVE_POSITION_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "servo_subsystem.hpp"
#include "aruwsrc/robot/engineer/algorithms/engineer_transforms.hpp"

namespace aruwsrc::engineer::servo
{
class ServoMovePositionCommand : public tap::control::Command
{
public:
    ServoMovePositionCommand(ServoSubsystem& subsystem, const tap::algorithms::transforms::Transform &cubeStore1ToCube);

    const char* getName() const override { return "Servo Move Position Command"; }

    bool isReady() override { return true; }

    void initialize() override {}

    void execute() override {}

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    ServoSubsystem& subsystem;
    const tap::algorithms::transforms::Transform &vtmTotr;
    const tap::algorithms::transforms::Transform &cubeStore2ToCube;
};
}  // namespace aruwsrc::engineer
#endif