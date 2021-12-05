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

#ifndef __FRICTION_WHEEL_ROTATE_COMMAND_HPP__
#define __FRICTION_WHEEL_ROTATE_COMMAND_HPP__

#include "tap/control/command.hpp"

namespace aruwsrc
{
namespace launcher
{
class FrictionWheelSubsystem;

/**
 * Command which sets a given friction wheel subsystem to a set projectile
 * launch speed (in m/s).
 */
class FrictionWheelRotateCommand : public tap::control::Command
{
public:
    /**
     * @param[in] frictionWheels The friction wheel subsystem that this command "owns".
     * @param[in] launchSpeed The desired launch speed of the projectiles coming out of the friction
     *      wheels, in m/s.
     */
    FrictionWheelRotateCommand(FrictionWheelSubsystem* frictionWheels, float launchSpeed);

    void initialize() override;

    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "friction wheel rotate"; }

private:
    FrictionWheelSubsystem* frictionWheelSubsystem;

    float launchSpeed;
};

}  // namespace launcher

}  // namespace aruwsrc

#endif
