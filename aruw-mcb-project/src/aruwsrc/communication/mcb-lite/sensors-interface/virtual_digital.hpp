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

#include "aruwsrc/communication/mcb-lite/message_types.hpp"

namespace aruwsrc::virtualMCB
{

// Struct of message coming from MCBLite
struct DigitalInputPinMessage
{
    bool BPinValue;
    bool CPinValue;
    bool DPinValue;
    bool ButtonPinValue;
} modm_packed;

// Structs of messages being sent to MCBLite
struct DigitalOutputPinMessage
{
    bool EPinValue;
    bool FPinValue;
    bool GPinValue;
    bool HPinValue;
    bool LaserPinValue;
} modm_packed;

struct DigitalPinModeMessage
{
    uint8_t BPinMode;
    uint8_t CPinMode;
    uint8_t DPinMode;
    uint8_t ButtonPinMode;
} modm_packed;

class VirtualDigital : public tap::gpio::Digital
{
    friend class SerialMCBLite;

public:
    VirtualDigital();

    // Don't call this please, shouldn't be used
    void init(){};

    /**
     * By default input pins are floating. Configure them to have a pull-up
     * or pull-down resistor here.
     *
     * @param[in] pin the InputPin to configure.
     * @param[in] mode the pull mode to be enabled.
     */
    void configureInputPullMode(InputPin pin, InputPullMode mode);

    /**
     * Sets the digital OutputPin either high or low.
     *
     * @param[in] pin the OutputPin to set.
     * @param[in] isSet `true` to send high, `false` to send low.
     */
    void set(OutputPin pin, bool isSet);

    /**
     * Reads from an InputPin.
     *
     * @param[in] pin the InputPin to read from.
     * @return `true` if the pin is pulled high and `false` otherwise.
     */
    bool read(InputPin pin) const;

private:
    bool outputPinE, outputPinF, outputPinG, outputPinH, outputPinLaser;

    uint8_t inputPinBMode, inputPinCMode, inputPinDMode, inputPinButtonMode;

    bool inputPinB, inputPinC, inputPinD, inputPinButton;

    bool hasNewMessageData = false;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(DigitalOutputPinMessage)>
        outputPinValuesMessage;
    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(DigitalPinModeMessage)>
        pinModesMessage;
};

}  // namespace aruwsrc::virtualMCB

#endif
