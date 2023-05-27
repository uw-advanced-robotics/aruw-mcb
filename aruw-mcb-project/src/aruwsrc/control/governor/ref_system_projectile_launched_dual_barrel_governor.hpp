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

#ifndef REF_SYSTEM_PROJECTILE_LAUNCHED_DUAL_BARREL_GOVERNOR_HPP_
#define REF_SYSTEM_PROJECTILE_LAUNCHED_DUAL_BARREL_GOVERNOR_HPP_

#include <optional>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

#include "aruwsrc\control\barrel-switcher\barrel_switcher_subsystem.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that halts a Comamnd's execution when the ref serial reports that a projectile has been
 * fired by one of the barrel mechanism IDs. This allows one to have a more consistent firing
 * frequency.
 *
 * @note Does not govern whether or not the governed Command can start execution.
 */
class RefSystemProjectileLaunchedDualBarrelGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    /**
     * @param[in] refSerial Global RefSerial object.
     * @param[in] barrelSwitcher Identifies the barrel (left or right) currently being used to launch projectiles.
     * @param[in] barrelMechanismIdLeft The ref system barrel ID of the left barrel in the dual barrel subsystem
     *  to check if a projectile has been fired.
     * @param[in] barrelMechanismIdRight The ref system barrel ID of the right barrel in the dual barrel subsystem
     *  to check if a projectile has been fired.
     */
    RefSystemProjectileLaunchedDualBarrelGovernor(
        tap::communication::serial::RefSerial &refSerial,
        aruwsrc::control::BarrelSwitcherSubsystem &barrelSwitcher,
        tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismIdLeft,
        tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismIdRight)
        : refSerial(refSerial),
          barrelSwitcher(barrelSwitcher),
          barrelMechanismIdLeft(barrelMechanismIdLeft),
          barrelMechanismIdRight(barrelMechanismIdRight)
    {
    }

    bool isReady() final
    {
        lastProjectileLaunchTime = getRecentProjectileLaunchTime();
        return true;
    }

    bool isFinished() final
    {
        return lastProjectileLaunchTime.has_value() &&
               getRecentProjectileLaunchTime().value() != lastProjectileLaunchTime.value();
    }

private:
    tap::communication::serial::RefSerial &refSerial;
    aruwsrc::control::BarrelSwitcherSubsystem &barrelSwitcher;
    tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismIdLeft;
    tap::communication::serial::RefSerialData::Rx::MechanismID barrelMechanismIdRight;
    std::optional<uint32_t> lastProjectileLaunchTime = 0;

    inline std::optional<uint32_t> getRecentProjectileLaunchTime()
    {
        if (!refSerial.getRefSerialReceivingData())
        {
            return std::nullopt;
        }
        
        if ((barrelSwitcher.getBarrelState() != BarrelState::USING_LEFT_BARREL ||
             refSerial.getRobotData().turret.launchMechanismID != barrelMechanismIdLeft) &&
            (barrelSwitcher.getBarrelState() != BarrelState::USING_RIGHT_BARREL ||
             refSerial.getRobotData().turret.launchMechanismID != barrelMechanismIdRight))
        {
            return lastProjectileLaunchTime;
        }

        //if using left and recieving left, or using right and recieving right
        return refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;
    }
};
}  // namespace aruwsrc::control::governor

#endif  // REF_SYSTEM_PROJECTILE_LAUNCHED_DUAL_BARREL_GOVERNOR_HPP_
