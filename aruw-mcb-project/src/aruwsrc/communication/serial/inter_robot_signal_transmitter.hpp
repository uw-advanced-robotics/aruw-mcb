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

#ifndef INTER_ROBOT_SIGNAL_TRANSMITTER_HPP_
#define INTER_ROBOT_SIGNAL_TRANSMITTER_HPP_

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

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::communication::serial
{
/**
 * An inter-robot signal transmitter sends messages to a specified list of targets
 */
template <typename MSG_TYPE_ENUM, uint8_t NUM_MSG_TYPES>
class InterRobotSignalMessageTransmitter : public modm::pt::Protothread
{
    static_assert(std::is_enum<MSG_TYPE_ENUM>(), "MSG_TYPE_ENUM must be an enum.");
    static_assert(NUM_MSG_TYPES <= 32, "Only 32 message types maximum allowed.");
    static_assert(NUM_MSG_TYPES >= 1, "There must at least be 1 message type.");

public:
    inline InterRobotSignalMessageTransmitter(
        tap::Drivers &drivers,
        std::vector<tap::communication::serial::RefSerialData::RobotId> targetIds,
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
                    robotToRobotMessage.dataAndCRC16[0] = nextMessageType;

                    for (currentTargetIdIdx = 0; currentTargetIdIdx < targetIds.size();
                         currentTargetIdIdx++)
                    {
                        PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
                            &robotToRobotMessage,
                            messageId,
                            refSerial.getRobotIdBasedOnCurrentRobotTeam(
                                targetIds[currentTargetIdIdx]),
                            2));
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

    inline void queueMessage(MSG_TYPE_ENUM type)
    {
        queuedMessageTypeBitmap |= (1 << static_cast<uint8_t>(type));
    }

private:
    tap::communication::serial::RefSerial &refSerial;

#ifdef ENV_UNIT_TESTS
public:
    tap::mock::RefSerialTransmitterMock refSerialTransmitter;

private:
#else
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
#endif

    std::vector<tap::communication::serial::RefSerialData::RobotId> targetIds;
    uint16_t messageId;

    uint8_t currentTargetIdIdx = 0;
    uint8_t nextMessageType = 0;
    uint32_t queuedMessageTypeBitmap = 0;
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    /**
     * @brief Sets nextMessageType to next requested message in queuedMessageTypeBitmap.
     * @return true if there is a next message and false if there are none
     */
    inline bool getNextMessageToSend()
    {
        if (queuedMessageTypeBitmap == 0)
        {
            return false;
        }

        // otherwise, iterate through message types until you find one that is queued
        auto getNextMessageType = [](uint8_t type) { return (type + 1) % NUM_MSG_TYPES; };

        while ((queuedMessageTypeBitmap & (1 << nextMessageType)) == 0)
        {
            nextMessageType = getNextMessageType(nextMessageType);
        }

        return true;
    }
};
}  // namespace aruwsrc::communication::serial

#endif  // INTER_ROBOT_SIGNAL_TRANSMITTER_HPP_
