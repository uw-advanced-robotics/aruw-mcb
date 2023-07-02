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

#ifndef DUAL_BARREL_HEAT_LIMIT_GOVERNOR_HPP_
#define DUAL_BARREL_HEAT_LIMIT_GOVERNOR_HPP_

#include <cassert>

#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"
#include "aruwsrc/control/governor/heat_tracker.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor for turrets with two barrels for a BarrelSwitcherSubsystem
 * that blocks Commands from running if the referee-reported heat limit for the currently
 * used barrel is too high or neither of the barrels are in position to fire. Use to
 * avoid running commands that cause ref-system overheating and properly schedule commands
 * that require the barrels to be in position.
 */
class HeatLimitDualBarrelSwitcherGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    HeatLimitDualBarrelSwitcherGovernor(
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDLeft,
        const tap::communication::serial::RefSerialData::Rx::MechanismID
            firingSystemMechanismIDRight,
        const uint16_t heatLimitBuffer,
        aruwsrc::control::BarrelSwitcherSubsystem &barrelSwitcher)
        : heatTrackerLeft(drivers, firingSystemMechanismIDLeft, heatLimitBuffer),
          heatTrackerRight(drivers, firingSystemMechanismIDRight, heatLimitBuffer),
          barrelSwitcher(barrelSwitcher)
    {
    }

    bool isReady() final
    {
        if (!barrelSwitcher.isInPosition()) return false;
        if (barrelSwitcher.getBarrelState() == BarrelState::USING_LEFT_BARREL)
        {
            return heatTrackerLeft.enoughHeatToLaunchProjectile();
        }
        else if (barrelSwitcher.getBarrelState() == BarrelState::USING_RIGHT_BARREL)
        {
            return heatTrackerRight.enoughHeatToLaunchProjectile();
        }
        return false;
    }

    bool isFinished() final
    {
        if (!barrelSwitcher.isInPosition()) return true;
        if (barrelSwitcher.getBarrelState() == BarrelState::USING_LEFT_BARREL)
        {
            return !heatTrackerLeft.enoughHeatToLaunchProjectile();
        }
        else if (barrelSwitcher.getBarrelState() == BarrelState::USING_RIGHT_BARREL)
        {
            return !heatTrackerRight.enoughHeatToLaunchProjectile();
        }
        return true;
    }

private:
    aruwsrc::control::governor::HeatTracker heatTrackerLeft;
    aruwsrc::control::governor::HeatTracker heatTrackerRight;
    aruwsrc::control::BarrelSwitcherSubsystem &barrelSwitcher;
};
}  // namespace aruwsrc::control::governor

#endif  // DUAL_BARREL_HEAT_LIMIT_GOVERNOR_HPP_
