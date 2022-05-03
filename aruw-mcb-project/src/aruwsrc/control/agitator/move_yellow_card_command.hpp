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
class MoveYellowCardCommand : public tap::control::Command
{
public:
    MoveYellowCardCommand(
        aruwsrc::Drivers *drivers,
        tap::control::Subsystem &dependentSubsystem,
        tap::control::Command &moveCommandNormal,
        tap::control::Command &moveCommandWhenYellowCarded);

    virtual const char *getName() const { return "Move yellow card command"; }
    virtual bool isReady();
    virtual void initialize() = 0;
    virtual void execute() = 0;
    virtual void end(bool interrupted) = 0;
    virtual bool isFinished() const = 0;

private:
    aruwsrc::Drivers *drivers;
    tap::control::Command &moveCommandNormal;
    tap::control::Command &moveCommandWhenYellowCarded;

    bool initializedWhenYellowCarded = false;

    bool isYellowCarded() const;
};
}  // namespace aruwsrc::agitator

#endif  // MOVE_YELLOW_CARD_COMMAND_HPP_
