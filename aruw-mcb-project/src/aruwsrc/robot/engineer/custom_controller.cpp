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

#include "custom_controller.hpp"

namespace aruwsrc::engineer
{
void CustomController::initialize()
{
    drivers->uart.init<CUSTOM_CONTROLLER_RX_UART_PORT, CUSTOM_CONTROLLER_BAUD_RATE>();
}

void CustomController::messageReceiveAndReadCallback(const ReceivedSerialMessage& message)
{
    if (message.messageType == CUSTOM_CONTROLLER_MESSAGE_TYPE)
    {
        if (message.header.dataLength < sizeof(ControllerInfoWire)) return;
        ControllerInfoWire wire;
        memcpy(&wire, &message.data, sizeof(ControllerInfoWire));

        controller.x = wire.x / INT_TO_FLOAT_CONV;
        controller.y = wire.y / INT_TO_FLOAT_CONV;
        controller.z = wire.z / INT_TO_FLOAT_CONV;

        controller.yaw = wire.yaw / INT_TO_FLOAT_CONV;
        controller.pitch = wire.pitch / INT_TO_FLOAT_CONV;
        controller.roll = wire.roll / INT_TO_FLOAT_CONV;

        controller.joystick_axes = wire.joystick_axes;
        controller.sensitivity = wire.sensitivity;
        controller.buttons_trigger_suction = wire.buttons_trigger_suction;

        lastRead = tap::arch::clock::getTimeMilliseconds();
        connected = true;
    }
}

void CustomController::update()
{
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT)
    {
        reset();
        connected = false;
    }
}

void CustomController::reset() { controller = {}; }

bool CustomController::isConnected() const { return connected; }
}  // namespace aruwsrc::engineer