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

#ifndef PAUSE_COMMAND_GOVERNOR_HPP_
#define PAUSE_COMMAND_GOVERNOR_HPP_

#include <cstdint>

#include "tap/architecture/timeout.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

namespace aruwsrc::control::governor
{
/**
 * A governor that when triggered pauses the ability to schedule an associated Command for a
 * specified period.
 */
class PauseCommandGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] pauseCommandPeriod The period in ms during which the associatd Command should be
     * paused.
     */
    PauseCommandGovernor(uint32_t pauseCommandPeriod) : pauseCommandPeriod(pauseCommandPeriod)
    {
        timeout.restart(0);
    }

    bool isReady() override { return timeout.isExpired(); }

    bool isFinished() override { return false; }

    void initiatePause() { timeout.restart(pauseCommandPeriod); }

private:
    const uint32_t pauseCommandPeriod;

    tap::arch::MilliTimeout timeout;
};

}  // namespace aruwsrc::control::governor

#endif  // PAUSE_COMMAND_GOVERNOR_HPP_
