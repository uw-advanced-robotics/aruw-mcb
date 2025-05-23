/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ROBOT_ORBIT_MESSAGE_QUEUE_HPP_
#define ROBOT_ORBIT_MESSAGE_QUEUE_HPP_

#ifdef ENV_UNIT_TESTS
#include "tap/mock/ref_serial_transmitter_mock.hpp"
#else
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#endif

#include <type_traits>

#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "tap/drivers.hpp"

#include "modm/processing/protothread.hpp"

namespace aruwsrc::communication::serial
{
enum class RobotOrbitMessageType : uint8_t
{
    POSITION_UPDATE = 0,
    NUM_MESSAGE_TYPES
};

class RobotOrbitMessageQueue : public modm::pt::Protothread
{
    static_assert(
        static_cast<uint8_t>(RobotOrbitMessageType::NUM_MESSAGE_TYPES) <= 32,
        "Only 32 message types maximum allowed.");
    static_assert(
        static_cast<uint8_t>(RobotOrbitMessageType::NUM_MESSAGE_TYPES) >= 1,
        "There must at least be 1 message type.");

public:
    static constexpr size_t MAX_MESSAGE_DATA_SIZE = 113;

    inline RobotOrbitMessageQueue(
        tap::Drivers& drivers,
        std::vector<tap::communication::serial::RefSerialData::RobotId>& targetIds,
        uint16_t messageId)
        : refSerial(drivers.refSerial),
          refSerialTransmitter(&drivers),
          targetIds(targetIds),
          messageId(messageId)
    {
    }

    inline bool sendQueued()
    {
        PT_BEGIN();
        {
            while (true)
            {
                if (getNextMessageToSend())
                {
                    prepareMessageForTransmission();
                    for (currentTargetIdIdx = 0; currentTargetIdIdx < targetIds.size();
                         currentTargetIdIdx++)
                    {
                        PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                            &robotToRobotMessage,
                            messageId,
                            refSerial.getRobotIdBasedOnCurrentRobotTeam(
                                targetIds[currentTargetIdIdx]),
                            currentMessageLength));
                    }
                }
                else
                {
                    PT_YIELD();
                }
                queuedMessageTypeBitmap &= ~(1 << nextMessageType);
            }
        }
        PT_END();
    }

    inline void setMessageData(RobotOrbitMessageType type, const uint8_t* data, size_t length)
    {
        if (length > MAX_MESSAGE_DATA_SIZE - 1)
        {
            length = MAX_MESSAGE_DATA_SIZE - 1;
        }

        uint8_t typeValue = static_cast<uint8_t>(type);

        messageDataLength[typeValue] = length;
        for (size_t i = 0; i < length; i++)
        {
            messageData[typeValue][i] = data[i];
        }

        queuedMessageTypeBitmap |= (1 << typeValue);
    }

private:
    tap::communication::serial::RefSerial& refSerial;

#ifdef ENV_UNIT_TESTS
public:
    tap::mock::RefSerialTransmitterMock refSerialTransmitter;
#else
public:
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
#endif

private:
    std::vector<tap::communication::serial::RefSerialData::RobotId>& targetIds;
    uint16_t messageId;

    uint8_t currentTargetIdIdx = 0;
    uint8_t nextMessageType = 0;
    uint32_t queuedMessageTypeBitmap = 0;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    static constexpr uint8_t NUM_MSG_TYPES =
        static_cast<uint8_t>(RobotOrbitMessageType::NUM_MESSAGE_TYPES);
    uint8_t messageData[NUM_MSG_TYPES][MAX_MESSAGE_DATA_SIZE];
    size_t messageDataLength[NUM_MSG_TYPES] = {};
    size_t currentMessageLength = 0;

    inline bool getNextMessageToSend()
    {
        if (queuedMessageTypeBitmap == 0)
        {
            return false;
        }

        auto getNextMessageType = [](uint8_t type) { return (type + 1) % NUM_MSG_TYPES; };

        while ((queuedMessageTypeBitmap & (1 << nextMessageType)) == 0)
        {
            nextMessageType = getNextMessageType(nextMessageType);
        }

        return true;
    }

    inline void prepareMessageForTransmission()
    {
        robotToRobotMessage.dataAndCRC16[0] = nextMessageType;

        size_t dataLength = messageDataLength[nextMessageType];
        for (size_t i = 0; i < dataLength; i++)
        {
            robotToRobotMessage.dataAndCRC16[i + 1] = messageData[nextMessageType][i];
        }

        currentMessageLength = dataLength + 1;
    }
};

}  // namespace aruwsrc::communication::serial

#endif  // ROBOT_ORBIT_MESSAGE_QUEUE_HPP_