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

#ifndef VIRTUAL_DIGITAL_HPP
#define VIRTUAL_DIGITAL_HPP

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
        switch (pin)
        {
            case InputPin::B:
                BPinMode = mode;
                break;
            case InputPin::C:
                CPinMode = mode;
                break;
            case InputPin::D:
                DPinMode = mode;
                break;
            case InputPin::Button:
                ButtonPinMode = mode;
                break;
            default:
                break;
        }
        updateMessages();
    }

    void set(OutputPin pin, bool isSet)
    {
        switch (pin)
        {
            case OutputPin::E:
                EPinValue = isSet;
                break;
            case OutputPin::F:
                FPinValue = isSet;
                break;
            case OutputPin::G:
                GPinValue = isSet;
                break;
            case OutputPin::H:
                HPinValue = isSet;
                break;
            case OutputPin::Laser:
                LaserPinValue = isSet;
                break;
            default:
                break;
        }
        updateMessages();
    }

    bool read(InputPin pin) const
    {
        switch (pin)
        {
            case InputPin::B:
                return BPinValue;
            case InputPin::C:
                return CPinValue;
            case InputPin::D:
                return DPinValue;
            case InputPin::Button:
                return ButtonPinValue;
            default:
                return false;
        }
    }

private:
    void processDigitalMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        DigitalInputPinMessage* digitalMessage = (DigitalInputPinMessage*)(completeMessage.data);
        BPinValue = digitalMessage->BPinValue;
        CPinValue = digitalMessage->CPinValue;
        DPinValue = digitalMessage->DPinValue;
        ButtonPinValue = digitalMessage->ButtonPinValue;
    }

    void updateMessages()
    {
        DigitalOutputPinMessage outputPinMessageData;
        outputPinMessageData.EPinValue = EPinValue;
        outputPinMessageData.FPinValue = FPinValue;
        outputPinMessageData.GPinValue = GPinValue;
        outputPinMessageData.HPinValue = HPinValue;
        outputPinMessageData.LaserPinValue = LaserPinValue;
        memcpy(outputPinMessage.data, &outputPinMessageData, sizeof(DigitalOutputPinMessage));
        outputPinMessage.setCRC16();

        DigitalPinModeMessage pinModeMessageData;
        pinModeMessageData.BPinMode = (uint8_t)BPinMode;
        pinModeMessageData.CPinMode = (uint8_t)CPinMode;
        pinModeMessageData.DPinMode = (uint8_t)DPinMode;
        pinModeMessageData.ButtonPinMode = (uint8_t)ButtonPinMode;
        memcpy(pinModeMessage.data, &pinModeMessageData, sizeof(DigitalPinModeMessage));
        pinModeMessage.setCRC16();

        hasNewData = true;
    }

    InputPullMode BPinMode, CPinMode, DPinMode, ButtonPinMode;
    bool BPinValue, CPinValue, DPinValue, ButtonPinValue;
    bool EPinValue, FPinValue, GPinValue, HPinValue, LaserPinValue;

    DJISerial::DJISerial::SerialMessage<sizeof(DigitalOutputPinMessage)> outputPinMessage;
    DJISerial::DJISerial::SerialMessage<sizeof(DigitalPinModeMessage)> pinModeMessage;

    bool hasNewData = false;
};

}  // namespace aruwsrc::virtualMCB

#endif  // VIRTUAL_DIGITAL_HPP
