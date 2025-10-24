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

#ifndef FIRED_RECENTLY_GOVERNOR_HPP_
#define FIRED_RECENTLY_GOVERNOR_HPP_

#include <cassert>

#include "tap/architecture/clock.hpp"
#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/ref_system_constants.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that blocks commands from running if a shot has been fired recently.
 */
class FiredRecentlyGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param durationBuffer Time since last shot in milliseconds to run the command blocked.
     */
    FiredRecentlyGovernor(tap::Drivers* drivers, const uint32_t durationBuffer)
        : drivers(drivers),
          durationBuffer(durationBuffer)
    {
    }

    bool isReady() final { return enoughTimeSinceLastShot(); }

    bool isFinished() final { return !enoughTimeSinceLastShot(); }

private:
    tap::Drivers* drivers;

    const uint32_t durationBuffer;

    bool enoughTimeSinceLastShot() const
    {
        const auto currentTime = tap::arch::clock::getTimeMilliseconds();
        const auto lastShot =
            this->drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;

        return currentTime - lastShot > durationBuffer;
    }
};
}  // namespace aruwsrc::control::governor

#endif  //  FIRED_RECENTLY_GOVERNOR_HPP_
