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

#ifndef BARREL_SWITCH_COMMAND_HPP_
#define BARREL_SWITCH_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"
#include "aruwsrc/control/governor/heat_tracker.hpp"

namespace aruwsrc::control
{
class BarrelSwitchCommand : public tap::control::Command
{
public:
    enum SwitchingControlState : uint8_t
    {
        USING_RIGHT = 0,
        USING_LEFT = 1,
        AUTOMATIC = 2,
        NUM_STATES = 3
    };

    BarrelSwitchCommand(
        aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher,
        tap::Drivers& drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDLeft,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDRight,
        const uint16_t heatLimitBuffer)
        : barrelSwitcher(barrelSwitcher),
          heatTrackerLeft(drivers, firingSystemMechanismIDLeft, heatLimitBuffer),
          heatTrackerRight(drivers, firingSystemMechanismIDRight, heatLimitBuffer)
    {
        addSubsystemRequirement(barrelSwitcher);
    };

    void initialize() override;

    void execute() override;

    void setControlState(SwitchingControlState state);

    bool isFinished() const override;

    void end(bool interrupt) override;

    const char* getName() const override { return "barrel switch command"; };

private:
    aruwsrc::control::BarrelSwitcherSubsystem* barrelSwitcher;
    aruwsrc::control::governor::HeatTracker heatTrackerLeft;
    aruwsrc::control::governor::HeatTracker heatTrackerRight;
    BarrelSwitchCommand::SwitchingControlState controlState;
};  // class BarrelSwitchCommand
}  // namespace aruwsrc::control

#endif
