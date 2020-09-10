/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "serial_test_class.hpp"

namespace aruwlib
{
namespace serial
{
SerialTestClass::SerialTestClass() : DJISerial(Uart::UartPort::Uart2, true), messageId(0), i(0) {}

void SerialTestClass::messageReceiveCallback(const SerialMessage& completeMessage)
{
    messageId = completeMessage.sequenceNumber;
}

void SerialTestClass::sendMessage()
{
    this->txMessage.length = 1;
    this->txMessage.headByte = 0xa5;
    this->txMessage.sequenceNumber = i;
    this->txMessage.type = 4;
    this->txMessage.data[0] = 60;
    this->send();
    i++;
}

}  // namespace serial

}  // namespace aruwlib
