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

/**
 * A class that extends the shoot comprised command and defines the system parameters of the
 * comprised command. The constants are choosen for fast rotation speed for a soldier robot's
 * agitator.
 */
class MoveUnjamRefLimitedCommand : public tap::control::setpoint::MoveUnjamComprisedCommand
{
public:
    MoveUnjamRefLimitedCommand(
        aruwsrc::Drivers* drivers,
        AgitatorSubsystem* agitator17mm,
        float agitatorRotateAngle,
        float maxUnjamRotateAngle,
        uint32_t rotateTime,
        bool heatLimiting,
        float heatLimitBuffer);

    bool isReady() override;

    bool isFinished() const override;

private:
    aruwsrc::Drivers* drivers;

    const bool heatLimiting;
    const float heatLimitBuffer;
};  // class ShootFastComprisedCommand

}  // namespace aruwsrc::agitator

#endif  // MOVE_UNJAM_REF_LIMITED_COMMAND_HPP_
