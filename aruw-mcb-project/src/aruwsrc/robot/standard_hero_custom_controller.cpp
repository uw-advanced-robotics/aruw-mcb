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

#include "standard_hero_custom_controller.hpp"

namespace aruwsrc::engineer
{
void StandardHeroCustomController::initialize()
{
    drivers->uart.init<CUSTOM_CONTROLLER_RX_UART_PORT, CUSTOM_CONTROLLER_BAUD_RATE>();
}

void StandardHeroCustomController::messageReceiveAndReadCallback(const ReceivedSerialMessage& message)
{
    if (message.messageType == CUSTOM_CONTROLLER_MESSAGE_TYPE)
    {
        if (message.header.dataLength < sizeof(StandardHeroControllerInfoWire)) return;
        StandardHeroControllerInfoWire wire;
        memcpy(&wire, &message.data, sizeof(StandardHeroControllerInfoWire));

        controller.x = wire.x / INT_TO_FLOAT_CONV;
        controller.y = wire.y / INT_TO_FLOAT_CONV;
        controller.z = wire.z / INT_TO_FLOAT_CONV;
        for (int i = 0; i < NUM_BUTTONS; i++) {
            controller.buttons[i] = wire.buttons[i];
        }
        
        lastRead = tap::arch::clock::getTimeMilliseconds();
        connected = true;
    }
}

void StandardHeroCustomController::update()
{
    if (tap::arch::clock::getTimeMilliseconds() - lastRead > REMOTE_DISCONNECT_TIMEOUT)
    {
        reset();
        connected = false;
    }
}

void StandardHeroCustomController::reset() { controller = {}; }

bool StandardHeroCustomController::isConnected() const { return connected; }
}  // namespace aruwsrc::engineer