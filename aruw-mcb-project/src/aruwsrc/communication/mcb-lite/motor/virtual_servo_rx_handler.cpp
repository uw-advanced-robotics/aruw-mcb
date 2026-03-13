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
VirtualServoRxHandler::VirtualServoRxHandler(tap::Drivers* drivers) : drivers(drivers), servoOne(nullptr), servoTwo(nullptr){}

void VirtualServoRxHandler::attachReceiveHandler(VirtualServo* const servo, bool isServoOne)
{
    if (isServoOne) {
        servoOne = servo; // update to catch overriding servo
    } else {
        servoTwo = servo;
    }
}

void VirtualServoRxHandler::processServoFeedbackMessage(const DJISerial::ReceivedSerialMessage& completeMessage) {
    const ServoFeedbackMessage* message =
        reinterpret_cast<const ServoFeedbackMessage*>(completeMessage.data);

    if (servoOne != nullptr && servoOne->pin == message->pin) {
        servoOne->processServoUARTMessage(message->currentPwm, message->isRampTargetMet);
    } else if (servoTwo != nullptr && servoTwo->pin == message->pin) {
        servoTwo->processServoUARTMessage(message->currentPwm, message->isRampTargetMet);
    }
}

void VirtualServoRxHandler::removeServoHandler(const VirtualServo& servo)
{
    if (servoOne != nullptr && servoOne->pin == servo.pin) {
        servoOne = nullptr;
    } else if (servoTwo != nullptr && servoTwo->pin == servo.pin) {
        servoTwo = nullptr;
    }
}

}  // namespace aruwsrc::communication::mcb_lite::motor
