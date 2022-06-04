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

#include "friction_wheel_spin_user_limited_command.hpp"

#include "tap/algorithms/math_user_utils.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::launcher
{
FrictionWheelSpinUserLimitedCommand::FrictionWheelSpinUserLimitedCommand(
    aruwsrc::Drivers *drivers,
    FrictionWheelSubsystem *frictionWheels,
    float defaultLaunchSpeed,
    float speedLimit)
    : drivers(drivers),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      speedLimit(speedLimit),
      launchSpeed(0)
{
    modm_assert(drivers != nullptr, "FrictionWheelSpinUserLimitedCommand", "nullptr exception");
    assert(speedLimit >= 0.0f);
    assert(defaultLaunchSpeed >= 0.0f);
    addSubsystemRequirement(frictionWheels);
}

void FrictionWheelSpinUserLimitedCommand::execute()
{
    float wheel = drivers->remote.getWheel();

    // TODO: fix magic number
    launchSpeed -= wheel / 10000.0f;
    launchSpeed = tap::algorithms::limitVal<float>(launchSpeed, 0, 15);

    frictionWheels->setDesiredLaunchSpeed(launchSpeed);
}

}  // namespace aruwsrc::control::launcher
