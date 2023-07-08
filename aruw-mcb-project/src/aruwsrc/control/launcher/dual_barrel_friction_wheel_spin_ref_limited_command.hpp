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

#include "friction_wheel_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::launcher
{
/**
 * An extension of the FrictionWheelSpinRefLimitedCommand for a turret with
 * two barrels and a BarrelSwitcherSubystem.
 *
 * Commands some associated friction wheel subsystem to spin such that the subsystem launches
 * projectiles at either the maximum speed allowed by the referee system or a default
 * speed specified in the constructor.
 *
 * The command will rotate at the default speed if the referee system is offline or if the
 * `alwaysUseDefaultSpeed` flag is set to true in the constructor.
 */
class DualBarrelFrictionWheelSpinRefLimitedCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] frictionWheels The friction wheel subsystem that this command
     *      "owns".
     * @param[in] defaultLaunchSpeed A launch speed in m/s that the command
     *      will request the subsystem to launch projectiles at if the referee
     *      system is not connected.
     * @param[in] alwaysUseDefaultLaunchSpeed If true, uses `defaultLaunchSpeed`
     *      independent of the state of the referee system.
     * @param[in] leftBarrel The turret's left barrel whose associated referee system barrel speed
     *      limit will be used to determine the projectile launch speed.
     * @param[in] rightBarrel The turret's right barrel whose associated referee system barrel speed
     *      limit will be used to determine the projectile launch speed.
     * @param[in] barrelSwitcher the BarrelSwitcherSubystem associated with the two barrels
     *      used to determine the current position of the barrels
     */
    DualBarrelFrictionWheelSpinRefLimitedCommand(
        tap::Drivers *drivers,
        FrictionWheelSubsystem *frictionWheels,
        float defaultLaunchSpeed,
        bool alwaysUseDefaultLaunchSpeed,
        tap::communication::serial::RefSerialData::Rx::MechanismID leftBarrel,
        tap::communication::serial::RefSerialData::Rx::MechanismID rightBarrel,
        aruwsrc::control::barrel_switcher::BarrelSwitcherSubsystem &barrelSwitcher);

    void initialize() override {}

    void execute() override;

    void end(bool) override { frictionWheels->setDesiredLaunchSpeed(0); }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "friction wheel spin ref limited"; }

private:
    tap::Drivers *drivers;

    FrictionWheelSubsystem *frictionWheels;

    const float defaultLaunchSpeed;

    const bool alwaysUseDefaultLaunchSpeed;

    const tap::communication::serial::RefSerialData::Rx::MechanismID leftBarrel;
    const tap::communication::serial::RefSerialData::Rx::MechanismID rightBarrel;

    const aruwsrc::control::barrel_switcher::BarrelSwitcherSubsystem &barrelSwitcher;
};
}  // namespace aruwsrc::control::launcher

#endif  // DUAL_BARREL_FRICTION_WHEEL_SPIN_REF_LIMITED_COMMAND_HPP_
