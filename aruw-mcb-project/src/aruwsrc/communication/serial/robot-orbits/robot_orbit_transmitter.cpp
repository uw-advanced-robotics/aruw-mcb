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

#include "tap/architecture/endianness_wrappers.hpp"

namespace aruwsrc::communication::serial
{
void RobotOrbitTransmitter::OrbitMessageTransmitter::setMessageData(
    RobotOrbitMessageType type, 
    const uint8_t* data, 
    size_t length)
{
    
    tap::communication::serial::RefSerialData::Tx::RobotToRobotMessage tempMsg;
    
    if (length > sizeof(tempMsg.dataAndCRC16) - 1) {
        length = sizeof(tempMsg.dataAndCRC16) - 1;  // Ensure we don't overflow
    }
    std::memcpy(tempMsg.dataAndCRC16 + 1, data, length);
    queueMessage(type);
}

RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers,
    RobotOrbitStateProvider& stateProvider,
    RefSerial* refSerial)
    : drivers(drivers),
      stateProvider(stateProvider),
      odometry(nullptr),
      refSerial(refSerial),
      targetRobots(),
      messageTransmitter(*drivers, targetRobots, 0x200)
{
    refSerial->attachRobotToRobotMessageHandler(0x200, this);
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

    targetRobots.clear();
    targetRobots.push_back(allyRobot);

    static constexpr size_t MAX_MSG_SIZE = 113; 
    uint8_t messageBuffer[MAX_MSG_SIZE] = {0};
    
    auto robotPosition = odometry->getCurrentLocation2D();

    messageBuffer[0] = 0x01;
    
    size_t baseIndex = 1;

    uint16_t xPosScaled = static_cast<uint16_t>(robotPosition.getX() * STATIC_CAST_SCALE_FACTOR);
    uint16_t yPosScaled = static_cast<uint16_t>(robotPosition.getY() * STATIC_CAST_SCALE_FACTOR);
    uint16_t zPosScaled = 0;  // Assuming ground robot with z=0

    tap::arch::convertToLittleEndian(xPosScaled, &messageBuffer[baseIndex]);
    tap::arch::convertToLittleEndian(yPosScaled, &messageBuffer[baseIndex + 2]);
    tap::arch::convertToLittleEndian(zPosScaled, &messageBuffer[baseIndex + 4]);

    baseIndex += 6;

    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    uint8_t numVisionStates = stateProvider.getNumKnownVisionStates(visionStates);

    for (uint8_t i = 0; i < numVisionStates; i++)
    {
        if (baseIndex + 7 >= MAX_MSG_SIZE)
        {
            break;
        }

        messageBuffer[0] |= (1 << (i + 1));

        messageBuffer[baseIndex] = static_cast<uint8_t>(visionStates[i].robotId);
        baseIndex++;

        uint16_t xPosRobot = static_cast<uint16_t>(visionStates[i].xPos * STATIC_CAST_SCALE_FACTOR);
        uint16_t yPosRobot = static_cast<uint16_t>(visionStates[i].yPos * STATIC_CAST_SCALE_FACTOR);
        uint16_t zPosRobot = static_cast<uint16_t>(visionStates[i].zPos * STATIC_CAST_SCALE_FACTOR);

        tap::arch::convertToLittleEndian(xPosRobot, &messageBuffer[baseIndex]);
        tap::arch::convertToLittleEndian(yPosRobot, &messageBuffer[baseIndex + 2]);
        tap::arch::convertToLittleEndian(zPosRobot, &messageBuffer[baseIndex + 4]);

        baseIndex += 6;
    }

    uint32_t timestamp = tap::arch::clock::getTimeMilliseconds();
    tap::arch::convertToLittleEndian(timestamp, &messageBuffer[baseIndex]);
    baseIndex += 4;

    messageTransmitter.setMessageData(
        RobotOrbitMessageType::POSITION_UPDATE,
        messageBuffer,
        baseIndex);

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

    const uint8_t* data = message.data + sizeof(tap::communication::serial::RefSerialData::Tx::InteractiveHeader);
    
    uint8_t headerTOC = data[0];
    size_t baseIndex = 1;

    if (headerTOC & 0x01)
    {
        uint16_t xPosScaled, yPosScaled, zPosScaled;
        tap::arch::convertFromLittleEndian(&xPosScaled, &data[baseIndex]);
        tap::arch::convertFromLittleEndian(&yPosScaled, &data[baseIndex + 2]);
        tap::arch::convertFromLittleEndian(&zPosScaled, &data[baseIndex + 4]);

        float xPos = static_cast<float>(xPosScaled) / STATIC_CAST_SCALE_FACTOR;
        float yPos = static_cast<float>(yPosScaled) / STATIC_CAST_SCALE_FACTOR;
        float zPos = static_cast<float>(zPosScaled) / STATIC_CAST_SCALE_FACTOR;

        RobotState allyRobotState;
        allyRobotState.robotId = allyRobot;
        allyRobotState.xPos = xPos;
        allyRobotState.yPos = yPos;
        allyRobotState.zPos = zPos;

        baseIndex += 6;

        for (uint8_t i = 1; i < MAX_TRACKED_ROBOTS + 1; i++)
        {
            if (!(headerTOC & (1 << i)))
            {
                continue;
            }

            RefSerialData::RobotId robotId = static_cast<RefSerialData::RobotId>(data[baseIndex]);
            baseIndex++;

            uint16_t xPosRobot, yPosRobot, zPosRobot;
            tap::arch::convertFromLittleEndian(&xPosRobot, &data[baseIndex]);
            tap::arch::convertFromLittleEndian(&yPosRobot, &data[baseIndex + 2]);
            tap::arch::convertFromLittleEndian(&zPosRobot, &data[baseIndex + 4]);

            float xRobot = static_cast<float>(xPosRobot) / STATIC_CAST_SCALE_FACTOR;
            float yRobot = static_cast<float>(yPosRobot) / STATIC_CAST_SCALE_FACTOR;
            float zRobot = static_cast<float>(zPosRobot) / STATIC_CAST_SCALE_FACTOR;

            RobotState newState;
            newState.robotId = robotId;
            newState.xPos = xRobot;
            newState.yPos = yRobot;
            newState.zPos = zRobot;

            baseIndex += 6;

            stateProvider.updateFromAlly(robotId, newState);
        }

        uint32_t timestamp;
        tap::arch::convertFromLittleEndian(&timestamp, &data[baseIndex]);

        allyRobotState.timestamp = timestamp;
        stateProvider.updateFromAlly(allyRobot, allyRobotState);
    }
}

void RobotOrbitTransmitter::update() 
{ 
    sendRobotStates();
    messageTransmitter.sendQueued(); 
}

}  // namespace aruwsrc::communication::serial
