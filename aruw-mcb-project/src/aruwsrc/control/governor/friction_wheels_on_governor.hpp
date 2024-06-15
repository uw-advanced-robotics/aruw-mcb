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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc/control/agitator/agitator_subsystem.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that allows one to gate a command from running when the actual, average friction wheel
 * speed isn't above a certain threshold.
 *
 * Useful for disallowing the agitator from rotating while friction wheels are not on.
 */
class FrictionWheelsOnGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] frictionWheel Reference to the friction wheel subsystem being used in the
     * governor's behavior.
     */
    FrictionWheelsOnGovernor(aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheel)
        : frictionWheel(frictionWheel)
    {
    }

    bool isReady() final
    {
        return !tap::algorithms::compareFloatClose(
                   frictionWheel.getDesiredFrictionWheelSpeed(),
                   0.0f,
                   1) &&
               frictionWheel.getCurrentFrictionWheelSpeed() >=
                   frictionWheel.getDesiredFrictionWheelSpeed() *
                       MINIMUM_SPEED_THRESHOLD_FRACTION &&
               frictionWheel.getCurrentFrictionWheelSpeed() <=
                   frictionWheel.getDesiredFrictionWheelSpeed() * MAXIMUM_SPEED_THRESHOLD_FRACTION;
    }

    bool isFinished() final { return !isReady(); }

private:
    aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheel;
#if defined(TARGET_HERO_PERSEUS)
    static constexpr float MINIMUM_SPEED_THRESHOLD_FRACTION = 0.95;
#else
    static constexpr float MINIMUM_SPEED_THRESHOLD_FRACTION = 0.9;
#endif
    static constexpr float MAXIMUM_SPEED_THRESHOLD_FRACTION = 1.02;
};
}  // namespace aruwsrc::control::governor

#endif  // FRICTION_WHEELS_ON_GOVERNOR_HPP_
