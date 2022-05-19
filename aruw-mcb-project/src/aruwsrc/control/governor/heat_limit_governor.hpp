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

#ifndef HEAT_LIMIT_COMMAND_GOVERNOR_HPP_
#define HEAT_LIMIT_COMMAND_GOVERNOR_HPP_

#include <cassert>

#include "tap/control/command_governor_interface.hpp"

#include "aruwsrc/drivers.hpp"

namespace aruwsrc::control::governor
{
class HeatLimitGovernor : public tap::control::CommandGovernorInterface
{
public:
    HeatLimitGovernor(
        aruwsrc::Drivers &drivers,
        const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID,
        const uint16_t heatLimitBuffer)
        : drivers(drivers),
          firingSystemMechanismID(firingSystemMechanismID),
          heatLimitBuffer(heatLimitBuffer)
    {
        // firing system mech ID must be valid
        assert(
            firingSystemMechanismID ==
                tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_1 ||
            firingSystemMechanismID ==
                tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_17MM_2 ||
            firingSystemMechanismID ==
                tap::communication::serial::RefSerialData::Rx::MechanismID::TURRET_42MM);
    }

    bool isReady() final { return enoughHeatToLaunchProjectile(); }

    bool isFinished() final { return !enoughHeatToLaunchProjectile(); }

private:
    aruwsrc::Drivers &drivers;

    const tap::communication::serial::RefSerialData::Rx::MechanismID firingSystemMechanismID;

    const uint16_t heatLimitBuffer;

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
};
}  // namespace aruwsrc::control::governor

#endif  //  HEAT_LIMIT_COMMAND_GOVERNOR_HPP_
