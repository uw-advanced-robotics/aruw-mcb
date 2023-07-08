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

#include "barrel_switch_command.hpp"

namespace aruwsrc::control::barrel_switcher
{
// ============================================================================
BarrelSwitchCommand::BarrelSwitchCommand(
    tap::Drivers& drivers,
    BarrelSwitcherSubsystem& barrelSwitcher,
    uint16_t heatLimitBuffer)
    : drivers(drivers),
      barrelSwitcher(barrelSwitcher),
      heatLimitBuffer(heatLimitBuffer)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&barrelSwitcher));
}

// ============================================================================
void BarrelSwitchCommand::initialize() {}

// ============================================================================
void BarrelSwitchCommand::execute()
{
    if (barrelSwitcher.getCurBarrelMechId().has_value())
    {
        if (!canShootSafely() && !barrelSwitchRequested)
        {
            barrelSwitchRequested = true;
            barrelSwitcher.switchBarrel();
        }

        if (barrelSwitchRequested && barrelSwitcher.isBarrelAligned())
        {
            barrelSwitchRequested = false;
        }
    }
}

// ============================================================================
bool BarrelSwitchCommand::canShootSafely()
{
    auto& turretData(drivers.refSerial.getRobotData().turret);

    uint16_t heatLimit{};
    uint16_t curHeat{};

    using MechanismID = tap::communication::serial::RefSerial::Rx::MechanismID;

    auto barrel(barrelSwitcher.getCurBarrelMechId());

    if (barrel.has_value())
    {
        switch (*barrel)
        {
            case MechanismID::TURRET_17MM_1:
            {
                curHeat = turretData.heat17ID1;
                heatLimit = turretData.heatLimit17ID1;
                break;
            }
            case MechanismID::TURRET_17MM_2:
            {
                curHeat = turretData.heat17ID2;
                heatLimit = turretData.heatLimit17ID2;
                break;
            }
            case MechanismID::TURRET_42MM:
                break;
            default:
                break;
        }
    }

    return curHeat + heatLimitBuffer < heatLimit;
}
}  // namespace aruwsrc::control::barrel_switcher
