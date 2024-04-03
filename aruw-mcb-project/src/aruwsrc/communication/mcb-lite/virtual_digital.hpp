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

#ifndef VIRTUAL_DIGITAL_HPP_
#define VIRTUAL_DIGITAL_HPP_

#include "tap/communication/gpio/digital.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
class VirtualDigital : public tap::gpio::Digital
{
    friend class MCBLite;

public:
    VirtualDigital() : outputPinMessage(), pinModeMessage()
    {
        pinModeMessage.messageType = MessageTypes::DIGITAL_PIN_MODE_MESSAGE;
        outputPinMessage.messageType = MessageTypes::DIGITAL_OUTPUT_MESSAGE;
    }

    void configureInputPullMode(InputPin pin, InputPullMode mode)
    {
        pinMode[pin] = mode;
        updateMessages();
    }

    void set(OutputPin pin, bool isSet)
    {
        outputPinValue[pin] = isSet;
        updateMessages();
    }

    bool read(InputPin pin) const { return inputPinValue[pin]; }

private:
    void processDigitalMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        memcpy(inputPinValue, completeMessage.data, sizeof(inputPinValue));
    }

    void updateMessages()
    {
        memcpy(outputPinMessage.data, outputPinValue, sizeof(DigitalOutputPinMessage));
        outputPinMessage.setCRC16();

        memcpy(pinModeMessage.data, pinMode, sizeof(DigitalPinModeMessage));
        pinModeMessage.setCRC16();

        hasNewData = true;
    }

    bool inputPinValue[4];
    InputPullMode pinMode[4];
    bool outputPinValue[5];

    DJISerial::DJISerial::SerialMessage<sizeof(DigitalOutputPinMessage)> outputPinMessage;
    DJISerial::DJISerial::SerialMessage<sizeof(DigitalPinModeMessage)> pinModeMessage;

    bool hasNewData = false;
};

}  // namespace aruwsrc::virtualMCB

#endif  // VIRTUAL_DIGITAL_HPP_
