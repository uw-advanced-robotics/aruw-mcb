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
#include "tap/drivers.hpp"


namespace aruwsrc::standard
{
CustomController::CustomController(tap::Drivers *drivers):
     DJISerial(drivers, CUSTOM_CONTROLLER_RX_UART_PORT) {}

void CustomController::initialize()
{
    drivers->uart.init<CUSTOM_CONTROLLER_RX_UART_PORT, CUSTOM_CONTROLLER_BAUD_RATE>();
}

// seems like this isn't actually being called???
void CustomController::messageReceiveCallback(const ReceivedSerialMessage& message)
{
    counter2++;
    messageType = message.messageType;
    if (message.messageType == CUSTOM_CONTROLLER_MESSAGE_TYPE)
    {
        if (message.header.dataLength < sizeof(ControllerInfo)) return;
        ControllerInfo info;
        memcpy(&info, &message.data, sizeof(ControllerInfo));
        lastRead = tap::arch::clock::getTimeMilliseconds();
        connected = true;
    }
}

void CustomController::update()
{
    counter++;
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT)
    {
        //reset();
        connected = false;
    }
}

void CustomController::reset() { controller = {}; }

bool CustomController::isConnected() const { return connected; }
}  // namespace aruwsrc::standard