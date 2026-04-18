/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef STANDARD_HERO_CUSTOM_CONTROLLER_HPP_
#define STANDARD_HERO_CUSTOM_CONTROLLER_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc
{
using ReceivedSerialMessage = tap::communication::serial::DJISerial::ReceivedSerialMessage;
class StandardHeroCustomController : public tap::communication::serial::DJISerial
{
public:
    StandardHeroCustomController(tap::Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(StandardHeroCustomController)
    mockable ~StandardHeroCustomController() = default;

    mockable void initialize();

    mockable void messageReceiveCallback(const ReceivedSerialMessage &message) override;

    mockable void update();

    mockable void reset();

    mockable bool isConnected() const;

    mockable float getX() { return controller.x; }

    mockable float getY() { return controller.y; }

    mockable float getZ() { return controller.z; }

    mockable bool getKeyPressed(int index)
    {
        if (index >= 0 && index < NUM_BUTTONS) return controller.buttons[index];
        return false;
    }

private:
    static constexpr uint8_t KEY_OFFSET = 1;
    static constexpr uint16_t CUSTOM_CONTROLLER_MESSAGE_TYPE = 17;  // TODO set to actual value
    static constexpr int NUM_BUTTONS = 20;

    static constexpr tap::communication::serial::Uart::UartPort CUSTOM_CONTROLLER_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart6;
    static constexpr size_t CUSTOM_CONTROLLER_BAUD_RATE = 115200; 
    static constexpr float INT_TO_FLOAT_CONV = 1000.0f;
    static const int REMOTE_DISCONNECT_TIMEOUT = 100;

    struct StandardHeroControllerInfoWire
    {
        uint16_t x, y, z;
        uint8_t buttons[NUM_BUTTONS];
    } modm_packed;

    struct StandardHeroControllerInfo
    {
        float x, y, z;
        uint8_t buttons[NUM_BUTTONS];
    };

    tap::Drivers *drivers;
    StandardHeroControllerInfo controller;
    bool connected = false;
    uint32_t lastRead = 0;
};
}  // namespace aruwsrc

#endif