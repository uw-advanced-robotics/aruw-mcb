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
 * Commands some associated friction wheel subsystem to rotate such that the subsystem launches
 * projectiles at some desired speed. The launch speed is limited by the max friction wheel speed
 * that we receive from the referee system.
 */
class FrictionWheelSpinRefLimitedCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] frictionWheels The friction wheel subsystem that this command
     *      "owns".
     * @param[in] defaultLaunchSpeed A launch speed in m/s that the command
     *      will request the subsystem to spin at if the referee system is not
     *      connected.
     */
    FrictionWheelSpinRefLimitedCommand(
        aruwsrc::Drivers *drivers,
        aruwsrc::launcher::FrictionWheelSubsystem *frictionWheels,
        float defaultLaunchSpeed);

    void initialize() override {}

    void execute() override;

    void end(bool) override { frictionWheels->setDesiredLaunchSpeed(0); }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "ref serial friction wheel rotate"; }

private:
    aruwsrc::Drivers *drivers;

    aruwsrc::launcher::FrictionWheelSubsystem *frictionWheels;

    const float defaultLaunchSpeed;
};
}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_REF_SERIAL_RPM_CONTROL_COMMAND_
