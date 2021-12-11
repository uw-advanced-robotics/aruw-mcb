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

#ifndef SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_
#define SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_

#include "tap/architecture/timeout.hpp"
#include "tap/control/comprised_command.hpp"
#include "tap/control/setpoint/commands/move_unjam_comprised_command.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::agitator
{
class AgitatorSubsystem;
}

namespace aruwsrc::control::sentinel::firing
{
class SentinelSwitcherSubsystem;

/**
 * A command that rotates an agitator that feeds into two barrels. A switcher is
 * used to alternate between barrels based on the heat limit of the barrel.
 */
class SentinelRotateAgitatorCommand : public tap::control::ComprisedCommand
{
public:
    SentinelRotateAgitatorCommand(
        aruwsrc::Drivers* drivers,
        tap::control::setpoint::SetpointSubsystem* agitator,
        SentinelSwitcherSubsystem* switcher);

    const char* getName() const override { return "sentinel rotate agitator"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

private:
    static constexpr uint32_t SWITCH_BARREL_TIMEOUT = 100;
    static constexpr uint16_t BARREL_OVERHEAT_THRESHOLD = 30;
    static constexpr float AGITATOR_ROTATE_ANGLE = 3 * M_PI / 10;
    static constexpr uint32_t AGITATOR_ROTATE_TIME = 54; // @TODO This seems unreasonably low to me (Tenzin)
    static constexpr uint32_t AGITATOR_WAIT_AFTER_ROTATE_TIME = 0;
    static constexpr float AGITATOR_SETPOINT_TOLERANCE = M_PI / 50;
    static constexpr float AGITATOR_UNJAM_DISPLACEMENT = M_PI / 4;
    static constexpr float AGITATOR_UNJAM_THRESHOLD = 3 * AGITATOR_UNJAM_DISPLACEMENT / 4;
    static constexpr uint32_t AGITATOR_UNJAM_WAIT_TIME = 100;
    static constexpr uint_fast16_t AGITATOR_UNJAM_CYCLES = 3;

    aruwsrc::Drivers* drivers;
    tap::control::setpoint::SetpointSubsystem* agitator;
    SentinelSwitcherSubsystem* switcher;

    tap::control::setpoint::MoveUnjamComprisedCommand rotateAgitator;

    tap::arch::MilliTimeout switchBarrelTimeout;

    bool switchingBarrel = false;
};

}  // namespace aruwsrc::control::sentinel::firing

#endif  // SENTINEL_AGITATOR_SYSTEM_COMPRISED_COMMAND_HPP_
