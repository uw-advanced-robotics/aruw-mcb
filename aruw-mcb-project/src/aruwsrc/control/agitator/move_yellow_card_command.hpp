/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef MOVE_YELLOW_CARD_COMMAND_HPP_
#define MOVE_YELLOW_CARD_COMMAND_HPP_

#include "tap/control/command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
/**
 * A command that switches between running two commands based on whether or not the operator is
 * blinded by a yellow card.
 */
class MoveYellowCardCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers Reference to a drivers object
     * @param[in] dependentSubsystem The Subsystem that the Commands depend on
     * @param[in] normalCommand Command to run when the operator is not blinded by a yellow card.
     * @param[in] yellowCardCommand Command to run when the operator is blinded by a yellow card.
     *
     * @attention The normal command and yellow card command must both have the same dependent
     * Subsystem, and it must be the passed in dependentSubsystem.
     */
    MoveYellowCardCommand(
        const aruwsrc::Drivers &drivers,
        tap::control::Subsystem &dependentSubsystem,
        tap::control::Command &normalCommand,
        tap::control::Command &yellowCardCommand);

    const char *getName() const override { return "Move yellow card command"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    const aruwsrc::Drivers &drivers;
    tap::control::Command &normalCommand;
    tap::control::Command &yellowCardCommand;

    bool initializedWhenYellowCarded = false;
};
}  // namespace aruwsrc::agitator

#endif  // MOVE_YELLOW_CARD_COMMAND_HPP_
