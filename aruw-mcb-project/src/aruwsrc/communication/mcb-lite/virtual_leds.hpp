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

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
class VirtualLEDs : public tap::gpio::Leds
{
    friend class MCBLite;

public:
    VirtualLEDs() : ledStateMessage()
    {
        ledStateMessage.messageType = MessageTypes::LED_CONTROL_MESSAGE;
    }

    void set(LedPin pin, bool isSet)
    {
        ledState[pin] = isSet;
        updateMessage();
    }

private:
    void updateMessage()
    {
        memcpy(ledStateMessage.data, &ledState, sizeof(LEDControlMessage));
        ledStateMessage.setCRC16();
        hasNewData = true;
    }

    bool ledState[10];
    DJISerial::DJISerial::SerialMessage<sizeof(LEDControlMessage)> ledStateMessage;
    bool hasNewData = false;
};

}  // namespace aruwsrc::virtualMCB
#endif  // VIRTUAL_LEDS_HPP_
