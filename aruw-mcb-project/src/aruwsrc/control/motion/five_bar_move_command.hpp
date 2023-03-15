/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FIVE_BAR_MOVE_COMMAND_HPP_
#define FIVE_BAR_MOVE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "five_bar_motion_subsystem.hpp"

namespace aruwsrc::control::motion
{
class FiveBarMoveCommand : public tap::control::Command
{
public:
    FiveBarMoveCommand(
        tap::Drivers* drivers,
        FiveBarMotionSubsystem* fiveBarSubsystem,
        const MOTION_FUNCTIONS motion);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "spin tmotor"; };

private:
    tap::Drivers* drivers;
    FiveBarMotionSubsystem* fiveBarSubsystem;
    MOTION_FUNCTIONS motion = RETURN_TO_HOME;
    int16_t speed;
};  // class SpinMotorCommand
}  // namespace aruwsrc::control::motion

#endif  // FIVE_BAR_MOVE_COMMAND_HPP
