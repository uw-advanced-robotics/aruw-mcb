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

#ifndef DUAL_BARREL_FRICTION_WHEEL_SPIN_REF_LIMITED_COMMAND_HPP_
#define DUAL_BARREL_FRICTION_WHEEL_SPIN_REF_LIMITED_COMMAND_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/barrel-switcher/barrel_switcher_subsystem.hpp"

#include "friction_wheel_spin_ref_limited_command.hpp"
#include "friction_wheel_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::launcher
{
/**
 * An extension of the FrictionWheelSpinRefLimitedCommand for a turret with
 * two barrels and a BarrelSwitcherSubsystem.
 *
 * Commands some associated friction wheel subsystem to spin such that the subsystem launches
 * projectiles at either the maximum speed allowed by the referee system or a default
 * speed specified in the constructor.
 */
class DualBarrelFrictionWheelSpinRefLimitedCommand : public FrictionWheelSpinRefLimitedCommand
{
public:
    using BarrelMechanismID = tap::communication::serial::RefSerialData::Rx::MechanismID;

    DualBarrelFrictionWheelSpinRefLimitedCommand(
        tap::Drivers *drivers,
        FrictionWheelSubsystem *frictionWheels,
        float defaultLaunchSpeed,
        bool alwaysUseDefaultLaunchSpeed,
        aruwsrc::control::barrel_switcher::BarrelSwitcherSubsystem &barrelSwitcher);

    const char *getName() const override { return "friction wheel spin ref limited"; }

private:
    const aruwsrc::control::barrel_switcher::BarrelSwitcherSubsystem &barrelSwitcher;

    uint16_t getMaxBarrelSpeed() const override;
};
}  // namespace aruwsrc::control::launcher

#endif  // DUAL_BARREL_FRICTION_WHEEL_SPIN_REF_LIMITED_COMMAND_HPP_
