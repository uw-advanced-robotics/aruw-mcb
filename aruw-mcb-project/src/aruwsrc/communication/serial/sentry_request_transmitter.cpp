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

#include "sentry_request_transmitter.hpp"

#include "tap/drivers.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::serial
{
SentryRequestTransmitter::SentryRequestTransmitter(tap::Drivers *drivers)
    : drivers(drivers),
      refSerialTransmitter(drivers)
{
}

bool SentryRequestTransmitter::send()
{
    PT_BEGIN();

    while (true)
    {
        getNextMessageToSend();

        if ((queuedMessageType & (1 << static_cast<uint8_t>(lastSentMessage))) != 0)
        {
            robotToRobotMessage.graphicData = static_cast<uint8_t>(lastSentMessage);

            // TODO configure rest of message if required by message type, currently this is not
            // necessary

            PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                &robotToRobotMessage,
                SENTRY_REQUEST_ROBOT_ID,
                drivers->refSerial.getRobotIdBasedOnCurrentRobotTeam(
                    RefSerialData::RobotId::BLUE_SENTINEL)));

            queuedMessageType &= ~(1 << static_cast<uint8_t>(lastSentMessage));
        }
        else
        {
            PT_YIELD();
        }
    }

    PT_END();
}

void SentryRequestTransmitter::queueRequest(SentryRequestMessageType type)
{
    queuedMessageType |= (1 << static_cast<uint8_t>(type));
}
}  // namespace aruwsrc::communication::serial
