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

#ifndef MOVED_FAST_RECENTLY_GOVERNOR_HPP_
#define MOVED_FAST_RECENTLY_GOVERNOR_HPP_

#include <cassert>

#include "tap/architecture/clock.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/chassis/chassis_rel_drive.hpp"
#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/ref_system_constants.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that blocks commands if the robot needs to move quickly.
 */
class MovedFastRecentlyGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param speedBuffer Speed limit for the robot to move fast
     * @param durationBuffer Time since last moved too fast in milliseconds to run the command
     * blocked.
     */
    MovedFastRecentlyGovernor(
        aruwsrc::control::ControlOperatorInterface& operatorInterface,
        const float speedBuffer,
        const uint32_t durationBuffer)
        : operatorInterface(operatorInterface),
          speedBuffer(speedBuffer),
          durationBuffer(durationBuffer)
    {
    }

    bool isReady() final { return timeSinceLastFastMovement(); }

    bool isFinished() final { return !timeSinceLastFastMovement(); }

private:
    aruwsrc::control::ControlOperatorInterface& operatorInterface;

    const float speedBuffer;
    const uint32_t durationBuffer;

    uint32_t lastTimeTooFast = -1;

    bool timeSinceLastFastMovement()
    {
        const auto currentTime = tap::arch::clock::getTimeMilliseconds();

        float x = operatorInterface.getChassisXInput();
        float y = operatorInterface.getChassisYInput();

        if (pow(x, 2) + pow(y, 2) >= pow(speedBuffer, 2))
        {
            lastTimeTooFast = currentTime;
        }

        return currentTime - lastTimeTooFast > durationBuffer;
    }
};
}  // namespace aruwsrc::control::governor

#endif  //  MOVED_FAST_RECENTLY_GOVERNOR_HPP_
