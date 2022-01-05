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

#ifndef FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_
#define FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_

#include "tap/control/command.hpp"

#include "friction_wheel_subsystem.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::launcher
{
/**
 * Commands some associated friction wheel subsystem to spin such that the subsystem launches
 * projectiles at either the maximum speed allowed by the referee system or a default
 * speed specified in the constructor.
 *
 * The command will rotate at the default speed if the referee system is offline or if the
 * `alwaysUseDefaultSpeed` flag is set to true in the constructor.
 */
class FrictionWheelSpinRefLimitedCommand : public tap::control::Command
{
public:
    /**
     * An enum that represents the 3 different barrel types, either the 1st or 2nd 17mm barrels or
     * the 42mm barrel.
     */
    enum class Barrel
    {
        BARREL_17MM_1,
        BARREL_17MM_2,
        BARREL_42MM,
    };

    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] frictionWheels The friction wheel subsystem that this command
     *      "owns".
     * @param[in] defaultLaunchSpeed A launch speed in m/s that the command
     *      will request the subsystem to launch projectiles at if the referee
     *      system is not connected.
     * @param[in] alwaysUseDefaultLaunchSpeed If true, uses `defaultLaunchSpeed`
     *      independent of the state of the referee system.
     * @param[in] barrel The barrel whose associated referee system barrel speed
     *      limit will be used to determine the projectile launch speed.
     */
    FrictionWheelSpinRefLimitedCommand(
        aruwsrc::Drivers *drivers,
        FrictionWheelSubsystem *frictionWheels,
        float defaultLaunchSpeed,
        bool alwaysUseDefaultLaunchSpeed,
        Barrel barrel);

    void initialize() override {}

    void execute() override;

    void end(bool) override { frictionWheels->setDesiredLaunchSpeed(0); }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "friction wheel spin ref limited"; }

private:
    aruwsrc::Drivers *drivers;

    FrictionWheelSubsystem *frictionWheels;

    const float defaultLaunchSpeed;

    const bool alwaysUseDefaultLaunchSpeed;

    const Barrel barrel;
};
}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_
