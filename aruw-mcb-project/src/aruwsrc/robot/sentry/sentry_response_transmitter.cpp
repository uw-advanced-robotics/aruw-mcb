/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "sentry_response_transmitter.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::serial
{
SentryResponseTransmitter::SentryResponseTransmitter(tap::Drivers *drivers)
    : drivers(drivers),
      refSerialTransmitter(drivers)
{
}

bool SentryResponseTransmitter::send()
{
    PT_BEGIN();

    while (true)
    {
        getNextMessageToSend();

        if ((queuedMessageType & (1 << static_cast<uint8_t>(lastSentMessage))) != 0)
        {
            *reinterpret_cast<uint16_t*>(this->robotToRobotMessage.dataAndCRC16) = static_cast<uint8_t>(lastSentMessage);

            PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                &robotToRobotMessage,
                SENTRY_RESPONSE_MESSAGE_ID,
                drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    RefSerialData::RobotId::BLUE_HERO),
                2));

            PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                &robotToRobotMessage,
                SENTRY_RESPONSE_MESSAGE_ID,
                drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    RefSerialData::RobotId::BLUE_SOLDIER_1),
                2));

            PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                &robotToRobotMessage,
                SENTRY_RESPONSE_MESSAGE_ID,
                drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    RefSerialData::RobotId::BLUE_SOLDIER_2),  // just in case we field two standards
                2));

            queuedMessageType &= ~(1 << static_cast<uint8_t>(lastSentMessage));
        }
        else
        {
            PT_YIELD();
        }
    }

    PT_END();
}

void SentryResponseTransmitter::queueRequest(SentryResponseType type)
{
    queuedMessageType |= (1 << static_cast<uint8_t>(type));
}
}  // namespace aruwsrc::communication::serial