/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "inter_robot_transmitter.hpp"

namespace aruwsrc::communication::inter_robot_comm
{
InterRobotTransmitter::InterRobotTransmitter(
    RefSerial* refSerial,
    RefSerialTransmitter* refSerialTransmitter)
    : refSerial(refSerial),
      refSerialTransmitter(refSerialTransmitter)
{
    refSerial->attachRobotToRobotMessageHandler(MSG_ID, this);
}

bool InterRobotTransmitter::sendMessage()
{
    PT_BEGIN()
    // Send the message using the refSerialTransmitter
    while (true)
    {
        PT_WAIT_UNTIL(timer.execute());
        targetId = getAllyRobotId();
        memcpy(&robotToRobotMessage.dataAndCRC16[0], &message, sizeof(Message));

        PT_CALL(refSerialTransmitter
                    ->sendRobotToRobotMsg(&robotToRobotMessage, MSG_ID, targetId, sizeof(Message)));
        ptLoopCount++;
    }
    PT_END();
}

void InterRobotTransmitter::operator()(const DJISerial::ReceivedSerialMessage& message)
{
    memcpy(
        &incomingMessage,
        &message.data[sizeof(RefSerialData::Tx::InteractiveHeader)],
        sizeof(Message));
    parsedMessageCount++;
}

RefSerialTransmitter::RobotId InterRobotTransmitter::getAllyRobotId() const
{
    const auto& robotData = refSerial->getRobotData();
    if (robotData.robotId == RefSerialData::RobotId::INVALID)
    {
        return RefSerialData::RobotId::INVALID;
    }

    bool isBlue = RefSerial::isBlueTeam(robotData.robotId);
    if (isBlue)
    {
        return (robotData.robotId == RefSerialData::RobotId::BLUE_HERO)
                   ? RefSerialData::RobotId::BLUE_SOLDIER_3
                   : RefSerialData::RobotId::BLUE_HERO;
    }
    else
    {
        return (robotData.robotId == RefSerialData::RobotId::RED_HERO)
                   ? RefSerialData::RobotId::RED_SOLDIER_3
                   : RefSerialData::RobotId::RED_HERO;
    }
}

}  // namespace aruwsrc::communication::inter_robot_comm
