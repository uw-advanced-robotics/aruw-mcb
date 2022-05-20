/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef FRICTION_WHEELS_ON_GOVERNOR_HPP_
#define FRICTION_WHEELS_ON_GOVERNOR_HPP_

#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that allows one to gate a command from running when a turret limit switch based on the
 * depressed state of a limit switch.
 *
 * The governor has two possible behaviors:
 * 1. It will allow a command to be run when the limit switch is depressed.
 * 2. The opposite, it will allow the command to be run when the limit switch is released.
 *
 * This is specified by the LimitSwitchGovernorBehavior.
 */
class FrictionWheelsOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in]
     * @param[in] behavior The behavior of the governor, whether or not to allow Commands to run
     * when the limit switch is depressed or released.
     */
    FrictionWheelsOnGovernor(aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheel)
        : frictionWheel(frictionWheel)
    {
    }

    bool isReady() final
    {
        return frictionWheel.getCurrentFrictionWheelSpeed() >=
               aruwsrc::control::launcher::LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT[1].second / 2;
    }

    bool isFinished() final { return !isReady(); }

private:
    aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheel;
};
}  // namespace aruwsrc::control::governor

#endif  // LIMIT_SWITCH_DEPRESSED_GOVERNOR_HPP_
