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

#ifndef HEAT_LIMIT_GOVERNOR_HPP_
#define HEAT_LIMIT_GOVERNOR_HPP_

#include <cassert>

#include "aruwsrc/control/governor/heat_tracker.hpp"

#include "tap/control/governor/command_governor_interface.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that blocks Commands from running if the referee-reported heat limit is too high. Use to
 * avoid running commands that cause ref-system overheating.
 */
class HeatLimitGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    HeatLimitGovernor(
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const uint16_t heatLimitBuffer)
        : heatTracker(drivers, firingSystemMechanismID, heatLimitBuffer)
    {
    }

    bool isReady() final { return heatTracker.enoughHeatToLaunchProjectile(); }

    bool isFinished() final { return !heatTracker.enoughHeatToLaunchProjectile(); }

private:
    aruwsrc::control::governor::HeatTracker heatTracker;
};
}  // namespace aruwsrc::control::governor

#endif  //  HEAT_LIMIT_GOVERNOR_HPP_
