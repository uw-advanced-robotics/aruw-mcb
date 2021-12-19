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

#ifndef ROTATE_KICKER_COMMAND_HPP_
#define ROTATE_KICKER_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "kicker_agitator_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 * This command rotates the kicker subsystem, no jamming required so just uses a MoveCommand
 */
class RotateKickerCommand : public tap::control::Command
{
public:
    // Buffer from max heat limit in which limiting occurs, for hero 100 is one shot.
    static constexpr uint16_t HEAT_LIMIT_BUFFER = 100;

    RotateKickerCommand(
        aruwsrc::Drivers* drivers,
        KickerAgitatorSubsystem* kicker,
        bool heatLimiting);

    // Override for heat limiting logic
    bool isReady() override;

    void initialize() override;

    void execute() {}

    void end(bool) {}

    bool isFinished() const override;

    const char* getName() const override { return "Rotate kicker"; }

private:
    aruwsrc::Drivers* drivers;
    KickerAgitatorSubsystem* kicker;
    const bool heatLimiting;
};
}  // namespace aruwsrc::agitator

#endif  // ROTATE_KICKER_COMMAND_HPP_
