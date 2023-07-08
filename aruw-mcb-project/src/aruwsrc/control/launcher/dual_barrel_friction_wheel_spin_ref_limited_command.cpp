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

#include "dual_barrel_friction_wheel_spin_ref_limited_command.hpp"

#include <cassert>

#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#include "modm/architecture/interface/assert.hpp"

namespace aruwsrc::control::launcher
{
DualBarrelFrictionWheelSpinRefLimitedCommand::DualBarrelFrictionWheelSpinRefLimitedCommand(
    tap::Drivers *drivers,
    FrictionWheelSubsystem *frictionWheels,
    float defaultLaunchSpeed,
    bool alwaysUseDefaultLaunchSpeed,
    aruwsrc::control::barrel_switcher::BarrelSwitcherSubsystem &barrelSwitcher)
    : FrictionWheelSpinRefLimitedCommand(
          drivers,
          frictionWheels,
          defaultLaunchSpeed,
          alwaysUseDefaultLaunchSpeed,
          BarrelMechanismID::TURRET_17MM_1),  // placeholder
      barrelSwitcher(barrelSwitcher)
{
}

uint16_t DualBarrelFrictionWheelSpinRefLimitedCommand::getMaxBarrelSpeed() const
{
    uint16_t maxBarrelSpeed = 0;
    auto barrel(barrelSwitcher.getCurBarrelMechId());

    if (barrel.has_value())
    {
        switch (*barrel)
        {
            case BarrelMechanismID::TURRET_17MM_1:
                maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;
                break;
            case BarrelMechanismID::TURRET_17MM_2:
                maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID2;
                break;
            case BarrelMechanismID::TURRET_42MM:
                // this doesn't make sense
                RAISE_ERROR(drivers, "invalid mech id");
                break;
        }
    }

    return maxBarrelSpeed;
}

}  // namespace aruwsrc::control::launcher
