/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef RANDOM_MOVING_TARGET_COMMAND_HPP_
#define RANDOM_MOVING_TARGET_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "motor_subsystem.hpp"

using namespace aruwsrc::dart_target;

class RandomMovingTargetCommand : public tap::control::Command
{
public:
    explicit RandomMovingTargetCommand(
        MotorSubsystem* subsystem);

    void initialize() override {}

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "Random Moving Target Command"; }

private:
    MotorSubsystem* motorSubsystem;
    uint32_t startTime;
    uint8_t targetPos;
    bool targetSet;
};  // class RandomMovingTargetCommand

#endif  // RANDOM_MOVING_TARGET_COMMAND_HPP_
