/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef HEAT_TRACKER_HPP_
#define HEAT_TRACKER_HPP_

#include "tap/drivers.hpp"

namespace aruwsrc::control::governor
{
/**
 * Member of governors that keep track of a barrel's referee-reported heat and whether it can fire
 * another projectile within the heat limit.
 */
class HeatTracker
{
public:
    HeatTracker(
        tap::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const uint16_t heatLimitBuffer)
        : drivers(drivers),
          firingSystemMechanismID(firingSystemMechanismID),
          heatLimitBuffer(heatLimitBuffer)
    {
    }

    bool enoughHeatToLaunchProjectile() const
    {
        if (!drivers.refSerial.getRefSerialReceivingData())
        {
            return true;
        }

        const auto &robotData = drivers.refSerial.getRobotData();

        uint16_t heat = 0, heatLimit = 0;

        switch (firingSystemMechanismID)
        {
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1:
                heat = robotData.turret.heat17ID1;
                heatLimit = robotData.turret.heatLimit17ID1;
                break;
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2:
                heat = robotData.turret.heat17ID2;
                heatLimit = robotData.turret.heatLimit17ID2;
                break;
            case tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM:
                heat = robotData.turret.heat42;
                heatLimit = robotData.turret.heatLimit42;
                break;
            default:
                // don't perform heat limiting
                heat = 0;
                heatLimit = heatLimitBuffer;
        }

        const bool heatBelowLimit = heat + heatLimitBuffer <= heatLimit;

        return !tap::communication::serial::RefSerial::heatAndLimitValid(heat, heatLimit) ||
               heatBelowLimit;
    }

private:
    tap::Drivers &drivers;

    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    const uint16_t heatLimitBuffer;
};
}  // namespace aruwsrc::control::governor

#endif  // HEAT_TRACKER_HPP_
