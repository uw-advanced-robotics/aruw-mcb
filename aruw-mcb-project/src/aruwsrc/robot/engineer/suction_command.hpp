/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SUCTION_COMMAND_HPP_
#define SUCTION_COMMAND_HPP_

#include <vector>

#include "tap/communication/gpio/digital.hpp"
#include "tap/control/command.hpp"

#include "suction_subsystem.hpp"

namespace aruwsrc::engineer::wrist
{
class SuctionCommand : public tap::control::Command
{
public:
    SuctionCommand(SuctionSubsystem &subsystem, bool suckState, bool releaseState)
        : subsystem(subsystem),
          suckState(suckState),
          releaseState(releaseState)
    {
        addSubsystemRequirement(&subsystem);
    }

    void initialize() override {}

    void execute() override { subsystem.set(suckState, releaseState); };

    void end(bool) override { subsystem.refreshSafeDisconnect(); };

    bool isFinished() const override { return false; };

    const char *getName() const override { return "Digital Out Command"; }

private:
    SuctionSubsystem &subsystem;
    bool suckState, releaseState;
};  // class SuctionCommand

}  // namespace aruwsrc::engineer::wrist

#endif  // SUCTION_COMMAND_HPP_