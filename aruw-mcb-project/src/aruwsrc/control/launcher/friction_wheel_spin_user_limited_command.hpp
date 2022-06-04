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

#ifndef FRICTION_WHEEL_SPIN_USER_LIMITED_COMMAND_HPP_
#define FRICTION_WHEEL_SPIN_USER_LIMITED_COMMAND_HPP_

#include "tap/communication/serial/ref_serial_data.hpp"
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
 * projectiles at a user determined speed. Speed is controlled using the thumbwheel
 * on the DT7 remote control.
 *
 * The command will rotate at the default speed at startup.
 */
class FrictionWheelSpinUserLimitedCommand : public tap::control::Command
{
public:
    /**
     * @param[in] drivers A pointer to the global drivers object.
     * @param[in] frictionWheels The friction wheel subsystem that this command
     *      "owns".
     * @param[in] defaultLaunchSpeed A launch speed in m/s that the command
     *      will request the subsystem to launch projectiles at if the referee
     *      system is not connected.
     * @param[in] speedLimit the max speed that can be requested
     */
    FrictionWheelSpinUserLimitedCommand(
        aruwsrc::Drivers *drivers,
        FrictionWheelSubsystem *frictionWheels,
        float defaultLaunchSpeed,
        float speedLimit);

    void initialize() override { launchSpeed = defaultLaunchSpeed; }

    void execute() override;

    void end(bool) override { frictionWheels->setDesiredLaunchSpeed(0); }

    bool isFinished() const override { return false; }

    const char *getName() const override { return "friction wheel spin ref limited"; }

private:
    aruwsrc::Drivers *drivers;

    FrictionWheelSubsystem *frictionWheels;

    /**
     * Default launch speed to use when command is scheduled (m/s)
     */
    const float defaultLaunchSpeed;

    /**
     * Max speed to allow (m/s)
     */
    const float speedLimit;

    /**
     * Current target launch speed
     */
    float launchSpeed;
};
}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_SPIN_USER_LIMITED_COMMAND_HPP_
