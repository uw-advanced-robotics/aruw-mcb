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

#include "tap/drivers.hpp"

#include "modm/architecture/interface/assert.hpp"

namespace aruwsrc::control::launcher
{
DualBarrelFrictionWheelSpinRefLimitedCommand::DualBarrelFrictionWheelSpinRefLimitedCommand(
    tap::Drivers *drivers,
    FrictionWheelSubsystem *frictionWheels,
    float defaultLaunchSpeed,
    bool alwaysUseDefaultLaunchSpeed,
    tap::communication::serial::RefSerialData::Rx::MechanismID leftBarrel,
    tap::communication::serial::RefSerialData::Rx::MechanismID rightBarrel,
    aruwsrc::control::BarrelSwitcherSubsystem &barrelSwitcher)
    : drivers(drivers),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      alwaysUseDefaultLaunchSpeed(alwaysUseDefaultLaunchSpeed),
      leftBarrel(leftBarrel),
      rightBarrel(rightBarrel),
      barrelSwitcher(barrelSwitcher)
{
    modm_assert(drivers != nullptr, "FrictionWheelSpinRefLimitedCommand", "nullptr exception");
    addSubsystemRequirement(frictionWheels);
}

void DualBarrelFrictionWheelSpinRefLimitedCommand::execute()
{
    if (alwaysUseDefaultLaunchSpeed || !drivers->refSerial.getRefSerialReceivingData())
    {
        frictionWheels->setDesiredLaunchSpeed(defaultLaunchSpeed);
    }
    else
    {
        uint16_t maxBarrelSpeed = 0;
        tap::communication::serial::RefSerialData::Rx::MechanismID barrel;

        if (barrelSwitcher.getBarrelState() == aruwsrc::control::BarrelState::USING_LEFT_BARREL)
        {
            barrel = leftBarrel;
        }
        else if (
            barrelSwitcher.getBarrelState() == aruwsrc::control::BarrelState::USING_RIGHT_BARREL)
        {
            barrel = rightBarrel;
        }
        else
        {
            frictionWheels->setDesiredLaunchSpeed(0);
            return;
        }

        if (barrelSwitcher.getBarrelState() != aruwsrc::control::BarrelState::IDLE)
        {
            switch (barrel)
            {
                case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1:
                    maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;
                    break;
                case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2:
                    maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID2;
                    break;
                case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM:
                    maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit42;
                    break;
            }
            frictionWheels->setDesiredLaunchSpeed(maxBarrelSpeed);
        }
    }
}

}  // namespace aruwsrc::control::launcher
