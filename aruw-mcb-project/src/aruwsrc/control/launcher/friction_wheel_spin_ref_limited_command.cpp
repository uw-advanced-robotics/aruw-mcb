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

#include "tap/drivers.hpp"

#include "modm/architecture/interface/assert.hpp"

#include "launcher_constants.hpp"

namespace aruwsrc::control::launcher
{
FrictionWheelSpinRefLimitedCommand::FrictionWheelSpinRefLimitedCommand(
    tap::Drivers *drivers,
    FrictionWheelInterface *frictionWheels,
    float defaultLaunchSpeed,
    bool alwaysUseDefaultLaunchSpeed,
    tap::communication::serial::RefSerialData::Rx::MechanismID barrel)
    : drivers(drivers),
      frictionWheels(frictionWheels),
      defaultLaunchSpeed(defaultLaunchSpeed),
      alwaysUseDefaultLaunchSpeed(alwaysUseDefaultLaunchSpeed),
      barrel(barrel)
{
    modm_assert(drivers != nullptr, "FrictionWheelSpinRefLimitedCommand", "nullptr exception");
    addSubsystemRequirement(frictionWheels);
}

#if defined(TARGET_FLYWHEEL_TESTING)
FlywheelRpms testingRpms = flywheelTestingRpms;
#endif

void FrictionWheelSpinRefLimitedCommand::execute()
{
    // @todo dubious
    if (alwaysUseDefaultLaunchSpeed || !drivers->refSerial.getRefSerialReceivingData())
    {
        frictionWheels->setDesiredLaunchSpeed(defaultLaunchSpeed);
    }
    else
    {
        frictionWheels->setDesiredLaunchSpeed(LAUNCHER_SPEED);
    }

#if defined(TARGET_FLYWHEEL_TESTING)
    frictionWheels->changeWheelVelocityState(0, true);
    frictionWheels->changeWheelVelocityState(1, true);
    frictionWheels->changeWheelVelocityState(2, true);
    frictionWheels->changeWheelVelocityState(3, true);
    frictionWheels->changeWheelVelocityState(4, true);

    if (defaultLaunchSpeed == 0)
    {
        // left
        frictionWheels->setIndividualVelocity(0, 0);
        // right
        frictionWheels->setIndividualVelocity(1, 0);
        // lower
        frictionWheels->setIndividualVelocity(2, 0);
        // upper
        frictionWheels->setIndividualVelocity(3, 0);
        // small upper
        frictionWheels->setIndividualVelocity(4, 0);
    }
    else
    {
        // left
        frictionWheels->setIndividualVelocity(0, testingRpms.leftRpm);
        // right
        frictionWheels->setIndividualVelocity(1, testingRpms.rightRpm);
        // lower
        frictionWheels->setIndividualVelocity(2, testingRpms.lowerRpm);
        // upper
        frictionWheels->setIndividualVelocity(3, testingRpms.upperRpm);
        // small upper
        frictionWheels->setIndividualVelocity(4, testingRpms.smallUpperRpm);
    }
#endif
}

}  // namespace aruwsrc::control::launcher
