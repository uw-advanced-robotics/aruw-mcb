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

#ifndef MOVE_UNJAM_REF_LIMITED_COMMAND_HPP_
#define MOVE_UNJAM_REF_LIMITED_COMMAND_HPP_

#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
class AgitatorSubsystem;

struct MoveUnjamRefLimitedCommandConfig
{
    float moveDisplacement = 0.0f;
    uint32_t moveTime = 0;
    uint32_t pauseAfterMoveTime = 0;
    bool setToTargetOnEnd = true;
    float setpointTolerance = 0.0f;
    float unjamDisplacement = 0.0f;
    float unjamThreshold = 0.0f;
    uint32_t maxUnjamWaitTime = 0;
    uint_fast16_t unjamCycleCount = 0;
    bool heatLimiting = true;
    float heatLimitBuffer = 0.0f;
};

/**
 * A command that will attempt to rotate an agitator a set amount and unjam if it
 * encounters a jam. This command has the option to be heat limited (in-game "heat")
 */
class MoveUnjamRefLimitedCommand : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
    /**
     * @note: All parameters except for `heatLimiting` and `heatLimitBuffer` are
     * passed directly to the `tap::control::setpoint::MoveUnjamComprisedCommand`
     * constructor, so see that class for what those parameters do.
     *
     * @param[in] heatLimiting if `true` this command will only schedule when the
     *      heat of 17mm barrel 1 is below the buffer.
     * @param[in] heatLimitBuffer If current_barrel_heat + heatLimitBuffer > barrel_heat_limit
     *      then command will not be scheduled. i.e.: How close you can get to the
     *      heat limit before the command won't be scheduled.
     */
    MoveUnjamRefLimitedCommand(
        aruwsrc::Drivers* drivers,
        tap::control::setpoint::SetpointSubsystem* setpointSubsystem,
        const MoveUnjamRefLimitedCommandConfig config);

    bool isReady() override;

    bool isFinished() const override;

private:
    aruwsrc::Drivers* drivers;

    // If `true` heat limiting is enabled, disabled otherwise
    const bool heatLimiting;
    // If current_barrel_heat + heatLimitBuffer > barrel_heat_limit then command
    // will not be scheduled. (How close to heat limit barrel can get before command
    // will stop scheduling). Should be _at least_ the heat value of a single launched
    // projectile if you don't want to overheat from the next shot.
    const float heatLimitBuffer;
};  // class ShootFastComprisedCommand

}  // namespace aruwsrc::agitator

#endif  // MOVE_UNJAM_REF_LIMITED_COMMAND_HPP_
