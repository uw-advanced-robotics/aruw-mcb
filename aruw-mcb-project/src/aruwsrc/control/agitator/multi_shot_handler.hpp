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

#ifndef MULTI_SHOT_HANDLER_HPP_
#define MULTI_SHOT_HANDLER_HPP_

#include "tap/control/hold_repeat_command_mapping.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator {
class MultiShotHandler
{
public:
    enum ShooterState : uint8_t
    {
        SINGLE = 0,
        BURST,
        FULL_AUTO,
        NUM_SHOOTER_STATES,
    };

    MultiShotHandler(tap::control::HoldRepeatCommandMapping *commandMapping, int burstCount);

    void setShooterState(ShooterState state);

    ShooterState getShooterState() const { return state; }

private:
    tap::control::HoldRepeatCommandMapping *commandMapping;
    const int burstCount;
    ShooterState state = SINGLE;
};

}

#endif  // MULTI_SHOT_HANDLER_HPP_
