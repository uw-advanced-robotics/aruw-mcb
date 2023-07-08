/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// Inspired by TAMU barrel switcher

#ifndef BARREL_SWITCH_COMMAND_HPP_
#define BARREL_SWITCH_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "barrel_switcher_subsystem.hpp"

namespace aruwsrc::control::barrel_switcher
{
class BarrelSwitchCommand : public tap::control::Command
{
public:
    BarrelSwitchCommand(
        tap::Drivers& drivers,
        BarrelSwitcherSubsystem& barrelSwitcher,
        uint16_t heatLimitBuffer);

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

    const char* getName() const override { return "barrel switch"; }

private:
    tap::Drivers& drivers;

    BarrelSwitcherSubsystem& barrelSwitcher;

    tap::arch::MilliTimeout calibrationRequestedTimeout;

    bool barrelCaliDoneFlag{false};

    bool barrelSwitchRequested{false};

    const uint16_t heatLimitBuffer;

    bool canShootSafely();
};

}  // namespace aruwsrc::control::barrel_switcher

#endif  // BARREL_SWITCH_COMMAND_HPP_
