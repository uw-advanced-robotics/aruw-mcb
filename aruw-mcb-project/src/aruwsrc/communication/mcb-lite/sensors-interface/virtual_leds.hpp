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

#ifndef VIRTUAL_LEDS_HPP_
#define VIRTUAL_LEDS_HPP_

#include "tap/communication/gpio/leds.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "aruwsrc/communication/mcb-lite/message_types.hpp"

namespace aruwsrc::virtualMCB
{
struct LEDControlMessage
{
    bool ALedOn;
    bool BLedOn;
    bool CLedOn;
    bool DLedOn;
    bool ELedOn;
    bool FLedOn;
    bool GLedOn;
    bool HLedOn;
    bool GreenLedOn;
    bool RedLedOn;
} modm_packed;

class VirtualLeds : public tap::gpio::Leds
{
    friend class SerialMCBLite;

public:
    VirtualLeds();

    void init();  // don't call this please

    void set(tap::gpio::Leds::LedPin led, bool isSet);

private:
    bool hasNewMessageData;

    bool ALedOn, BLedOn, CLedOn, DLedOn, ELedOn, FLedOn, GLedOn, HLedOn, GreenLedOn, RedLedOn;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(LEDControlMessage)>
        ledControlMessage;
};

}  // namespace aruwsrc::virtualMCB

#endif /* VIRTUAL_LEDS_HPP_ */
