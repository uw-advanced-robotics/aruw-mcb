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

#ifndef VIRTUAL_SERVO_RX_HANDLER_HPP_
#define VIRTUAL_SERVO_RX_HANDLER_HPP_

#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/communication/serial/uart.hpp"
#include "modm/architecture/interface/assert.h"
#include "virtual_servo.hpp"
#include "aruwsrc/communication/mcb-lite/message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::mcb_lite::motor
{
/**
 * This class is used to pass Servo messages from the virtual MCB to Servos
 */
class VirtualServoRxHandler
{
    friend class aruwsrc::communication::mcb_lite::MCBLite;
public:
    VirtualServoRxHandler(tap::Drivers* drivers);

    void attachReceiveHandler(VirtualServo* const servo, bool isServoOne);
    void processServoFeedbackMessage(const DJISerial::ReceivedSerialMessage& completeMessage);
    void removeServoHandler(const VirtualServo& servo);
private:
    tap::Drivers* drivers;
    VirtualServo* servoOne;
    VirtualServo* servoTwo;
};

}  // namespace aruwsrc::communication::mcb_lite::motor

#endif
