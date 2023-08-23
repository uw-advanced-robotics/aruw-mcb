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
    switch (pin)
    {
        case InputPin::B:
            pinModes[0] = static_cast<uint8_t>(mode);
            break;
        case InputPin::C:
            pinModes[1] = static_cast<uint8_t>(mode);
            break;
        case InputPin::D:
            pinModes[2] = static_cast<uint8_t>(mode);
            break;
        case InputPin::Button:
            pinModes[3] = static_cast<uint8_t>(mode);
            break;
        default:
            break;
    }
    updated = false;
	memcpy(pinModesMessage.data, pinModes, sizeof(pinModes));
	pinModesMessage.setCRC16();
}

void VirtualDigital::set(OutputPin pin, bool isSet)
{
    switch (pin)
    {
        case OutputPin::E:
            outputPinValues[0] = isSet;
            break;
        case OutputPin::F:
            outputPinValues[1] = isSet;
            break;
        case OutputPin::G:
            outputPinValues[2] = isSet;
            break;
        case OutputPin::H:
            outputPinValues[3] = isSet;
            break;
        case OutputPin::Laser:
            outputPinValues[4] = isSet;
            break;
        default:
            break;
    }
    updated = false;
	memcpy(outputPinValuesMessage.data, outputPinValues, sizeof(outputPinValues));
	outputPinValuesMessage.setCRC16();
}

bool VirtualDigital::read(InputPin pin) const
{
    switch (pin)
    {
        case InputPin::B:
            return inputPinValues[0];
        case InputPin::C:
            return inputPinValues[1];
        case InputPin::D:
            return inputPinValues[2];
        case InputPin::Button:
            return inputPinValues[3];
        default:
            return false;
    }
}

}  // namespace aruwsrc::virtualMCB
