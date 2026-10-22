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
#ifndef LIMIT_SWITCH_TRIGGER_HPP_
#define LIMIT_SWITCH_TRIGGER_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"
#include "tap/drivers.hpp"

#include "trigger_interface.hpp"

namespace aruwsrc::control::joint::homing::trigger
{
/**
 * Represents a "trigger" used by Homeable Subsystems to detect
 * through the limit switch when it is at an end of its axis.
 */
class LimitSwitchTrigger : public TriggerInterface
{
public:
    LimitSwitchTrigger(tap::communication::sensors::limit_switch::LimitSwitchInterface* limitSwitch)
        : limitSwitch(limitSwitch)
    {
    }
    bool isTriggered() { return limitSwitch->getLimitSwitchDepressed(); }

private:
    tap::communication::sensors::limit_switch::LimitSwitchInterface* limitSwitch;
};
}  // namespace aruwsrc::control::joint::homing::trigger

#endif  // LIMIT_SWITCH_TRIGGER_HPP_
