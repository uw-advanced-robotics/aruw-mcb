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

#ifndef REF_SYSTEM_PROJECTILE_LAUNCHED_GOVERNOR_HPP_
#define REF_SYSTEM_PROJECTILE_LAUNCHED_GOVERNOR_HPP_

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that halts a Comamnd's execution when the ref serial reports that a projectile has been
 * fired by one of the barrel mechanism IDs. This allows one to have a more consistent firing frequency.
 * 
 * @note Does not govern whether or not the governed Command can start execution.
 */
class RefSystemProjectileLaunchedGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] refSerial Global RefSerial object.
     * @param[in] barrelMechanismId The ref system barrel ID to check if a projectile has been fired.
     */
    RefSystemProjectileLaunchedGovernor(
        tap::communication::serial::RefSerial &refSerial,
        tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismId)
        : refSerial(refSerial),
          barrelMechanismId(barrelMechanismId)
    {
    }

    bool isReady() final
    {
        lastTime = getNewTime();
        return true;
    }

    bool isFinished() final { return getNewTime() != lastTime; }

private:
    tap::communication::serial::RefSerial &refSerial;
    tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismId;
    uint32_t lastTime = 0;

    inline uint32_t getNewTime()
    {
        if (refSerial.getRobotData().turret.launchMechanismID != barrelMechanismId ||
            refSerial.getRefSerialReceivingData())
        {
            return lastTime;
        }

        return refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;
    }
};
}  // namespace aruwsrc::control::governor

#endif  // REF_SYSTEM_PROJECTILE_LAUNCHED_GOVERNOR_HPP_
