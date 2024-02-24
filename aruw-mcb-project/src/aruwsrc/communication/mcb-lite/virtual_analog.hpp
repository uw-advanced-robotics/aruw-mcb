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

#ifndef VIRTUAL_ANALOG_HPP
#define VIRTUAL_ANALOG_HPP

#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
class VirtualAnalog : public tap::gpio::Analog
{
    friend class SerialMCBLite;

public:
    VirtualAnalog() {}

    uint16_t read(Pin pin) const
    {
        switch (pin)
        {
            case Pin::S:
                return SPinValue;
            case Pin::T:
                return TPinValue;
            case Pin::U:
                return UPinValue;
            case Pin::V:
                return VPinValue;
            case Pin::OledJoystick:
                return OLEDPinValue;
            default:
                return 0;
        }
    }

private:
    void processAnalogMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        AnalogInputPinMessage* analogMessage = (AnalogInputPinMessage*) (completeMessage.data);
        SPinValue = analogMessage->SPinValue;
        TPinValue = analogMessage->TPinValue;
        UPinValue = analogMessage->UPinValue;
        VPinValue = analogMessage->VPinValue;
        OLEDPinValue = analogMessage->OLEDPinValue;
    }

    uint16_t SPinValue, TPinValue, UPinValue, VPinValue, OLEDPinValue;

};

}  // namespace aruwsrc::virtualMCB

#endif
