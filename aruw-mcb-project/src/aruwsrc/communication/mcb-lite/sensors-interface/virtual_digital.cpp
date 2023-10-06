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

#include "virtual_digital.hpp"

namespace aruwsrc::virtualMCB
{

VirtualDigital::VirtualDigital() : outputPinValuesMessage(), pinModesMessage() {
	outputPinValuesMessage.messageType = DIGITAL_OUTPUT_MESSAGE;
	pinModesMessage.messageType = DIGITAL_PIN_MODE_MESSAGE;
}

void VirtualDigital::configureInputPullMode(InputPin pin, InputPullMode mode)
{
    uint8_t pinMode = static_cast<uint8_t>(mode);
    switch (pin)
    {
        case InputPin::B:
            inputPinBMode = pinMode;
            break;
        case InputPin::C:
            inputPinCMode = pinMode;
            break;
        case InputPin::D:
            inputPinDMode = pinMode;
            break;
        case InputPin::Button:
            inputPinButtonMode = pinMode;
            break;
        default:
            break;
    }
    pinModesMessage.data[pin] = pinMode;
	pinModesMessage.setCRC16();

    hasNewMessageData = true;
}

bool VirtualDigital::read(InputPin pin) const
{
    switch (pin)
    {
        case InputPin::B:
            return inputPinB;
        case InputPin::C:
            return inputPinC;
        case InputPin::D:
            return inputPinD;
        case InputPin::Button:
            return inputPinButton;
        default:
            return false;
    }
}

void VirtualDigital::set(OutputPin pin, bool isSet)
{
    switch (pin)
    {
        case OutputPin::E:
            outputPinE = isSet;
            break;
        case OutputPin::F:
            outputPinF = isSet;
            break;
        case OutputPin::G:
            outputPinG = isSet;
            break;
        case OutputPin::H:
            outputPinH = isSet;
            break;
        case OutputPin::Laser:
            outputPinLaser = isSet; 
            break;
        default:
            break;
    }
    outputPinValuesMessage.data[pin] = isSet;
	outputPinValuesMessage.setCRC16();
    
    hasNewMessageData = true;
}



}  // namespace aruwsrc::virtualMCB
