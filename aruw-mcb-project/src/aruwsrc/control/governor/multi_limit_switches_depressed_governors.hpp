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

#ifndef MULTI_LIMIT_SWITCHES_DEPRESSED_GOVERNOR_HPP_
#define MULTI_LIMIT_SWITCHES_DEPRESSED_GOVERNOR_HPP_

#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/control/governor/command_governor_interface.hpp"

namespace aruwsrc::control::governor
{
/**
 * Governor that gates a command based on the combined depressed state of two limit switches.
 * Both switches must satisfy the behavior condition for the governor to be ready.
 *
 * The governor has two possible behaviors:
 * 1. READY_WHEN_DEPRESSED: allows the command to run when BOTH limit switches are depressed.
 * 2. READY_WHEN_RELEASED: allows the command to run when BOTH limit switches are released.
 */
class MultiLimitSwitchDepressedGovernor : public tap::control::governor::CommandGovernorInterface
{
public:
    enum class LimitSwitchGovernorBehavior
    {
        READY_WHEN_DEPRESSED,
        READY_WHEN_RELEASED,
    };

    /**
     * @param[in] limitSwitch1 Reference to the first limit switch.
     * @param[in] limitSwitch2 Reference to the second limit switch.
     * @param[in] behavior Whether to allow commands to run when both switches are depressed or
     * released.
     */
    MultiLimitSwitchDepressedGovernor(
        tap::communication::sensors::limit_switch::LimitSwitchInterface& limitSwitch1,
        tap::communication::sensors::limit_switch::LimitSwitchInterface& limitSwitch2,
        LimitSwitchGovernorBehavior behavior)
        : limitSwitch1(limitSwitch1),
          limitSwitch2(limitSwitch2),
          behavior(behavior)
    {
    }

    bool isReady() final
    {
        bool bothDepressed =
            limitSwitch1.getLimitSwitchDepressed() && limitSwitch2.getLimitSwitchDepressed();
        switch (behavior)
        {
            case LimitSwitchGovernorBehavior::READY_WHEN_DEPRESSED:
                return bothDepressed;
            case LimitSwitchGovernorBehavior::READY_WHEN_RELEASED:
                return !bothDepressed;
            default:
                return false;
        }
    }

    bool isFinished() final { return !isReady(); }

private:
    tap::communication::sensors::limit_switch::LimitSwitchInterface& limitSwitch1;
    tap::communication::sensors::limit_switch::LimitSwitchInterface& limitSwitch2;
    LimitSwitchGovernorBehavior behavior;
};

}  // namespace aruwsrc::control::governor

#endif  // MULTI_LIMIT_SWITCHES_DEPRESSED_GOVERNOR_HPP_