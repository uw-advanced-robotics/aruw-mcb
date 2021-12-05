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

#include "friction_wheel_spin_ref_limited_command.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::launcher
{
FrictionWheelSpinRefLimitedCommand::FrictionWheelSpinRefLimitedCommand(
    aruwsrc::Drivers *drivers,
    aruwsrc::launcher::FrictionWheelSubsystem *frictionWheels,
    float defaultLaunchSpeed)
    : drivers(drivers),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed)
{
    modm_assert(drivers != nullptr, "FrictionWheelSpinRefLimitedCommand", "nullptr exception");
    addSubsystemRequirement(frictionWheels);
}

void FrictionWheelSpinRefLimitedCommand::execute()
{
    const uint16_t maxBarrelSpeed = drivers->refSerial.getRobotData().turret.barrelSpeedLimit17ID1;
    if (!drivers->refSerial.getRefSerialReceivingData())
    {
        frictionWheels->setDesiredLaunchSpeed(defaultLaunchSpeed);
    }
    else
    {
        frictionWheels->setDesiredLaunchSpeed(maxBarrelSpeed);
    }
}

}  // namespace aruwsrc::control::launcher
