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

#ifndef ENGINEER_CUSTOM_CONTROLLER_HPP_
#define ENGINEER_CUSTOM_CONTROLLER_HPP_

#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

namespace aruwsrc::engineer
{
using ReceivedSerialMessage = tap::communication::serial::DJISerial::ReceivedSerialMessage;
class CustomController : public tap::communication::serial::DJISerial
{
public:
    CustomController(tap::Drivers *drivers);
    DISALLOW_COPY_AND_ASSIGN(CustomController)
    mockable ~CustomController() = default;

    // TODO change to actual key names
    enum class Key
    {
        A,
        B,
        C,
        D,
        E
    };

    mockable void initialize();

    mockable void messageReceiveCallback(const ReceivedSerialMessage &message) override;

    mockable void update();

    mockable void reset();

    mockable bool isConnected() const;

    mockable float getX() { return controller.x; }

    mockable float getY() { return controller.y; }

    mockable float getZ() { return controller.z; }

    mockable float getYaw() { return controller.yaw; }

    mockable float getPitch() { return controller.pitch; }

    mockable float getRoll() { return controller.roll; }

    mockable float getJoystickX() { 
        return normalizedJoystickValue(controller.joystick_axes & JOYSTICK_X_MASK);
    }

    mockable float getJoystickY() { 
        return normalizedJoystickValue((controller.joystick_axes & JOYSTICK_Y_MASK) >> 10);
    }

    mockable bool isTriggerPressed() { return (controller.buttons_trigger_suction & TRIGGER_MASK); }

    mockable bool getKeyPressed(Key key)
    {
        return ((controller.buttons_trigger_suction >> (static_cast<int>(key) + KEY_OFFSET)) & 0x1);
    }

    mockable uint8_t getSensitivity() { return controller.sensitivity; }

    mockable bool suctionEnabled() { return controller.buttons_trigger_suction & SUCTION_MASK; }

private:
    static constexpr uint8_t KEY_OFFSET = 2;
    static constexpr uint16_t CUSTOM_CONTROLLER_MESSAGE_TYPE = 0x0302;
    static constexpr int SUCTION_MASK = 0x1;
    static constexpr int TRIGGER_MASK = 0x10;
    static constexpr int JOYSTICK_X_MASK = 0x3FF;
    static constexpr int JOYSTICK_Y_MASK = JOYSTICK_X_MASK << 10;

    static constexpr tap::communication::serial::Uart::UartPort CUSTOM_CONTROLLER_RX_UART_PORT =
        tap::communication::serial::Uart::UartPort::Uart6;          // TODO set to actual value
    static constexpr size_t CUSTOM_CONTROLLER_BAUD_RATE = 500'000;  // TODO set to actual value
    static constexpr float INT_TO_FLOAT_CONV = 1000.0f;
    static const int REMOTE_DISCONNECT_TIMEOUT = 100;

    struct ControllerInfoWire
    {
        uint32_t joystick_axes;
        int16_t x, y, z;
        int16_t yaw, pitch, roll;
        uint16_t sensitivity;
        uint8_t buttons_trigger_suction;
    } modm_packed;

    struct ControllerInfo
    {
        uint32_t joystick_axes;
        float x, y, z;
        float yaw, pitch, roll;
        uint16_t sensitivity;
        uint8_t buttons_trigger_suction;
    };

    // normalized between [-1, 1]
    float normalizedJoystickValue(int curVal) {
        return (curVal - 512.0f) / 511.0f;
    }

    tap::Drivers *drivers;
    ControllerInfo controller;
    bool connected = false;
    uint32_t lastRead = 0;

    //debug 
    uint16_t messageType;
};
}  // namespace aruwsrc::engineer

#endif