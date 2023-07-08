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
BarrelSwitchCommand::BarrelSwitchCommand(
    tap::Drivers& drivers,
    BarrelSwitcherSubsystem& barrelSwitcher)
    : drivers(drivers),
      barrelSwitcher(barrelSwitcher)
{
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(&barrelSwitcher));
}

void BarrelSwitchCommand::initialize() {}

void BarrelSwitchCommand::execute()
{
    // jank calibration
    auto bPressedRightSwitchDown(
        drivers.remote.keyPressed(tap::communication::serial::Remote::Key::G) &&
        drivers.remote.getSwitch(tap::communication::serial::Remote::Switch::RIGHT_SWITCH) ==
            tap::communication::serial::Remote::SwitchState::DOWN);

    // key pressed and calibration request timed out
    if (bPressedRightSwitchDown &&
        (calibrationRequestedTimeout.isStopped() || calibrationRequestedTimeout.isExpired()))
    {
        calibrationRequestedTimeout.restart(1000);
        barrelSwitcher.requestCalibration();
    }

    if (barrelSwitcher.getSide() != BarrelSide::CALIBRATING)
    {
        if (!canShootSafely() && !barrelSwitchRequested)
        {
            barrelSwitchRequested = true;
            barrelSwitcher.toggleSide();
        }

        if (barrelSwitchRequested && barrelSwitcher.isBarrelAligned())
        {
            barrelSwitchRequested = false;
        }
    }
}

void BarrelSwitchCommand::end(bool) {}

bool BarrelSwitchCommand::isReady() { return true; }

bool BarrelSwitchCommand::isFinished() const { return false; }

bool BarrelSwitchCommand::canShootSafely()
{
    auto& turretData(drivers.refSerial.getRobotData().turret);

    uint16_t heatLimit{};
    uint16_t curHeat{};

    using RefSerialRxData = tap::communication::serial::RefSerial::Rx;

    switch (barrelSwitcher.getCurrentBarrel())
    {
        case RefSerialRxData::MechanismID::TURRET_17MM_1:
        {
            curHeat = turretData.heat17ID1;
            heatLimit = turretData.heatLimit17ID1;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_17MM_2:
        {
            curHeat = turretData.heat17ID2;
            heatLimit = turretData.heatLimit17ID2;
            break;
        }
        case RefSerialRxData::MechanismID::TURRET_42MM:
            break;
        default:
            break;
    }

    return curHeat + 20 < heatLimit;
}
}  // namespace aruwsrc::control::barrel_switcher
