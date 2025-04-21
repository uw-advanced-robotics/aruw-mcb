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

#include "robot_orbit_transmitter.hpp"

namespace aruwsrc::communication::serial
{
RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers,
    RobotOrbitStateProvider& stateProvider,
    RefSerial* refSerial)
    : drivers(drivers),
      serialTransmitter(drivers),
      stateProvider(stateProvider),
      refSerial(refSerial)
{
}

RefSerialTransmitter::RobotId RobotOrbitTransmitter::getAllyRobotId() const
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

void RobotOrbitTransmitter::sendRobotStates()
{
    if (odometry == nullptr)
    {
        return;
    }
    RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot == RefSerialData::RobotId::INVALID)
    {
        return;
    }

    auto robotPosition = odometry->getCurrentLocation2D();

    RefSerialData::Tx::RobotToRobotMessage message{};
    message.dataAndCRC16[0] = 0;
    uint8_t baseIndex = 1;

    message.dataAndCRC16[baseIndex] =
        static_cast<uint8_t>(robotPosition.getX() * STATIC_CAST_SCALE_FACTOR);
    message.dataAndCRC16[baseIndex + 1] =
        static_cast<uint8_t>(robotPosition.getY() * STATIC_CAST_SCALE_FACTOR);

    baseIndex += 2;

    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    uint8_t numVisionStates = stateProvider.getNumKnownVisionStates(visionStates);

    for (uint8_t i = 0; i < numVisionStates; i++)
    {
        if (baseIndex + 4 >= static_cast<uint8_t>(sizeof(message.dataAndCRC16)))
        {
            break;
        }

        message.dataAndCRC16[0] |= (1 << (i + 1));

        message.dataAndCRC16[baseIndex] = static_cast<uint8_t>(visionStates[i].robotId);
        message.dataAndCRC16[baseIndex + 1] = visionStates[i].xPos;
        message.dataAndCRC16[baseIndex + 2] = visionStates[i].yPos;
        message.dataAndCRC16[baseIndex + 3] = visionStates[i].zPos;

        baseIndex += 4;
    }

    uint32_t timestamp = tap::arch::clock::getTimeMilliseconds();
    message.dataAndCRC16[baseIndex] = static_cast<uint8_t>(timestamp & 0xFF);
    message.dataAndCRC16[baseIndex + 1] = static_cast<uint8_t>((timestamp >> 8) & 0xFF);
    message.dataAndCRC16[baseIndex + 2] = static_cast<uint8_t>((timestamp >> 16) & 0xFF);
    message.dataAndCRC16[baseIndex + 3] = static_cast<uint8_t>((timestamp >> 24) & 0xFF);

    baseIndex += 4;

    serialTransmitter.sendRobotToRobotMsg(&message, 0x200, allyRobot, baseIndex);
}

void RobotOrbitTransmitter::operator()(const DJISerial::ReceivedSerialMessage& message)
{
    parseIncomingMessage(message);
}

void RobotOrbitTransmitter::parseIncomingMessage(const DJISerial::ReceivedSerialMessage& message)
{
    RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot == RefSerialData::RobotId::INVALID)
    {
        return;
    }

    uint8_t baseIndex = 1;
    const uint8_t* data = message.data;

    float xPos = static_cast<float>(data[baseIndex]) / STATIC_CAST_SCALE_FACTOR;
    float yPos = static_cast<float>(data[baseIndex + 1]) / STATIC_CAST_SCALE_FACTOR;

    RobotState allyRobotState;
    allyRobotState.robotId = allyRobot;
    allyRobotState.xPos = xPos;
    allyRobotState.yPos = yPos;
    allyRobotState.zPos = 0;

    baseIndex += 2;

    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++)
    {
        if (!(data[0] & (1 << (i + 1))))
        {
            continue;
        }

        RobotState newState;
        newState.robotId = static_cast<RefSerialData::RobotId>(data[baseIndex]);
        newState.xPos = data[baseIndex + 1];
        newState.yPos = data[baseIndex + 2];
        newState.zPos = data[baseIndex + 3];

        stateProvider.updateFromAlly(newState.robotId, newState);

        baseIndex += 4;
    }

    uint32_t timestamp = static_cast<uint32_t>(data[baseIndex]) |
                         (static_cast<uint32_t>(data[baseIndex + 1]) << 8) |
                         (static_cast<uint32_t>(data[baseIndex + 2]) << 16) |
                         (static_cast<uint32_t>(data[baseIndex + 3]) << 24);

    allyRobotState.timestamp = timestamp;

    for (uint8_t i = 0; i < MAX_TRACKED_ROBOTS; i++)
    {
        if (!(data[0] & (1 << (i + 1))))
        {
            continue;
        }

        RefSerialData::RobotId robotId = static_cast<RefSerialData::RobotId>(data[baseIndex]);
        RobotState updatedState = stateProvider.getRobotState(robotId);
        updatedState.timestamp = timestamp;
        stateProvider.updateFromAlly(robotId, updatedState);

        baseIndex += 4;
    }
    stateProvider.updateFromAlly(allyRobot, allyRobotState);
}

void RobotOrbitTransmitter::update()
{
    sendRobotStates();
}

}  // namespace aruwsrc::communication::serial
