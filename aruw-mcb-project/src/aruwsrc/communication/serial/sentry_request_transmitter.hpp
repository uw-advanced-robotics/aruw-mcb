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

#ifndef SENTRY_REQUEST_TRANSMITTER_HPP_
#define SENTRY_REQUEST_TRANSMITTER_HPP_

#ifdef ENV_UNIT_TESTS
#include "tap/mock/ref_serial_transmitter_mock.hpp"
#else
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#endif

#include "modm/architecture/interface/register.hpp"
#include "modm/processing/protothread.hpp"

#include "sentry_request_message_types.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
class SentryRequestTransmitter : public modm::pt::Protothread
{
public:
    SentryRequestTransmitter(tap::Drivers *drivers);

    bool send();

    void queueRequest(SentryRequestMessageType type);

private:
    tap::Drivers *drivers;

#ifdef ENV_UNIT_TESTS
public:
    tap::mock::RefSerialTransmitterMock refSerialTransmitter;

private:
#else
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
#endif

    SentryRequestMessageType lastSentMessage = SentryRequestMessageType::NONE;
    uint32_t queuedMessageType{};
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    inline void getNextMessageToSend()
    {
        // either no queued messages or lastSentMessage is the only message to send, return
        // w/o trying to find a new message
        if (queuedMessageType == 0)
        {
            return;
        }

        if ((queuedMessageType & ~(1 << static_cast<uint8_t>(lastSentMessage))) == 0)
        {
            return;
        }

        // otherwise, iterate through message types until you find one that is queued
        auto nextMessageType = [](SentryRequestMessageType type) {
            return static_cast<SentryRequestMessageType>(
                (static_cast<uint8_t>(type) + 1) %
                static_cast<uint8_t>(SentryRequestMessageType::NUM_MESSAGE_TYPES));
        };

        lastSentMessage = nextMessageType(lastSentMessage);

        while ((queuedMessageType & (1 << static_cast<uint8_t>(lastSentMessage))) == 0)
        {
            lastSentMessage = nextMessageType(lastSentMessage);
        }
    }
};
}  // namespace aruwsrc::communication::serial

#endif  // SENTRY_REQUEST_TRANSMITTER_HPP_
