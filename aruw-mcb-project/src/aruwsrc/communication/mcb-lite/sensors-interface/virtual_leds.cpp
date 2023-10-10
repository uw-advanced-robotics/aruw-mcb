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

#include "virtual_leds.hpp"

namespace aruwsrc::virtualMCB
{
VirtualLeds::VirtualLeds() { ledControlMessage.messageType = LED_CONTROL_MESSAGE; }

void VirtualLeds::set(tap::gpio::Leds::LedPin led, bool isSet)
{
    switch (led)
    {
        case tap::gpio::Leds::LedPin::A:
            ALedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::B:
            BLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::C:
            CLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::D:
            DLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::E:
            ELedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::F:
            FLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::G:
            GLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::H:
            HLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::Green:
            GreenLedOn = isSet;
            break;
        case tap::gpio::Leds::LedPin::Red:
            RedLedOn = isSet;
            break;
        default:
            break;
    }
    hasNewMessageData = true;

    ledControlMessage.data[led] = isSet;
    ledControlMessage.setCRC16();
}

}  // namespace aruwsrc::virtualMCB
