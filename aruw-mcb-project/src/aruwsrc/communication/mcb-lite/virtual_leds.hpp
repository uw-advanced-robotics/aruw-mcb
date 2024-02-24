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

#ifndef VIRTUAL_LED_HPP
#define VIRTUAL_LED_HPP

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
        switch (pin)
        {
            case LedPin::A:
                ALEDState = isSet;
                break;
            case LedPin::B:
                BLEDState = isSet;
                break;
            case LedPin::C:
                CLEDState = isSet;
                break;
            case LedPin::D:
                DLEDState = isSet;
                break;
            case LedPin::E:
                ELEDState = isSet;
                break;
            case LedPin::F:
                FLEDState = isSet;
                break;
            case LedPin::G:
                GLEDState = isSet;
                break;
            case LedPin::H:
                HLEDState = isSet;
                break;
            case LedPin::Green:
                GreenLEDState = isSet;
                break;
            case LedPin::Red:
                RedLEDState = isSet;
                break;
            default:
                break;
        }
        updateMessage();
    }

private:
    void updateMessage()
    {
        LEDControlMessage ledControlMessage;
        ledControlMessage.ALedOn = ALEDState;
        ledControlMessage.BLedOn = BLEDState;
        ledControlMessage.CLedOn = CLEDState;
        ledControlMessage.DLedOn = DLEDState;
        ledControlMessage.ELedOn = ELEDState;
        ledControlMessage.FLedOn = FLEDState;
        ledControlMessage.GLedOn = GLEDState;
        ledControlMessage.HLedOn = HLEDState;
        ledControlMessage.GreenLedOn = GreenLEDState;
        ledControlMessage.RedLedOn = RedLEDState;
        memcpy(ledStateMessage.data, &ledControlMessage, sizeof(LEDControlMessage));
        ledStateMessage.setCRC16();
        hasNewData = true;
    }

    bool ALEDState, BLEDState, CLEDState, DLEDState, ELEDState, FLEDState, GLEDState, HLEDState,
        GreenLEDState, RedLEDState;
    DJISerial::DJISerial::SerialMessage<sizeof(LEDControlMessage)> ledStateMessage;
    bool hasNewData = false;
};

}  // namespace aruwsrc::virtualMCB
#endif  // VIRTUAL_LED_HPP
