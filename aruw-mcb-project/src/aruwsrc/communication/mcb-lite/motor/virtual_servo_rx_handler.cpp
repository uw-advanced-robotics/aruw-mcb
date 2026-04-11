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

#include "virtual_servo_rx_handler.hpp"

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

namespace aruwsrc::communication::mcb_lite::motor
{
VirtualServoRxHandler::VirtualServoRxHandler(tap::Drivers* drivers) : drivers(drivers) {}

void VirtualServoRxHandler::attachReceiveHandler(VirtualServo* const servo)
{
    servos[static_cast<size_t>(servo->pin)] = servo;
}

void VirtualServoRxHandler::processServoFeedbackMessage(
    const DJISerial::ReceivedSerialMessage& completeMessage)
{
    const ServoFeedbackMessage* message =
        reinterpret_cast<const ServoFeedbackMessage*>(completeMessage.data);

    if (servos[static_cast<size_t>(message->pin)] != nullptr)
    {
        servos[static_cast<size_t>(message->pin)]->processServoUARTMessage(
            message->currentPwm,
            message->isRampTargetMet);
    }
}
}  // namespace aruwsrc::communication::mcb_lite::motor
