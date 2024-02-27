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
    friend class MCBLite;

public:
    VirtualAnalog() {}

    uint16_t read(Pin pin) const
    {
        return pinValues[pin];
    }

private:
    void processAnalogMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        memcpy(pinValues, completeMessage.data, sizeof(pinValues));
    }

    uint16_t pinValues[5];
};

}  // namespace aruwsrc::virtualMCB

#endif
