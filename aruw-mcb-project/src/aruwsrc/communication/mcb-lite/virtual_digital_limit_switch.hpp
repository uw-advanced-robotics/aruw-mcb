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
#ifndef VIRTUAL_DIGITAL_LIMIT_SWITCH_HPP_
#define VIRTUAL_DIGITAL_LIMIT_SWITCH_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/sensors/limit_switch/limit_switch_interface.hpp"

#include "aruwsrc/communication/mcb-lite/virtual_digital.hpp"

namespace aruwsrc::communication::mcb_lite
{
class VirtualDigitalLimitSwitch
    : public tap::communication::sensors::limit_switch::LimitSwitchInterface
{
public:
    VirtualDigitalLimitSwitch(
        VirtualDigital& digital,
        tap::gpio::Digital::InputPin pin,
        bool inverted = false)
        : digital(digital),
          pin(pin),
          inverted(inverted)
    {
    }

    bool getLimitSwitchDepressed() const override
    {
        if (inverted)
        {
            return !digital.read(pin);
        }
        else
        {
            return digital.read(pin);
        }
    }

private:
    VirtualDigital& digital;
    tap::gpio::Digital::InputPin pin;
    bool inverted;
};
}  // namespace aruwsrc::communication::mcb_lite
#endif