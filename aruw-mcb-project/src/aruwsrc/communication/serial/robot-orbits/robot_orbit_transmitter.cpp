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
RobotOrbitTransmitter::RobotOrbitTransmitter(
    tap::Drivers* drivers,
    RobotOrbitStateProvider& stateProvider,
    RefSerial* refSerial)
    : drivers(drivers),
      stateProvider(stateProvider),
      odometry(nullptr),
      refSerial(refSerial),
      refSerialTransmitter(drivers)
{
    refSerial->attachRobotToRobotMessageHandler(ROBOT_ORBIT_MSG_ID, this);

    memset(&outgoingData, 0, sizeof(PositionData));
    memset(&incomingData, 0, sizeof(PositionData));
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

void RobotOrbitTransmitter::updateState()
{
    memset(&outgoingData, 0, sizeof(PositionData));
    
    if (odometry != nullptr)
    {
        auto robotPosition = odometry->getCurrentLocation2D();
        
        outgoingData.positions[0].valid = true;
        outgoingData.positions[0].x = robotPosition.getX();
        outgoingData.positions[0].y = robotPosition.getY();
        outgoingData.positions[0].z = 0.0f;
        outgoingData.positions[0].timestamp = tap::arch::clock::getTimeMilliseconds();
    }
    
    RobotState visionStates[MAX_TRACKED_ROBOTS] = {};
    uint8_t numRobots = stateProvider.getNumKnownVisionStates(visionStates);
    
    for (uint8_t i = 0; i < numRobots && i < MAX_TRACKED_ROBOTS; i++)
    {
        outgoingData.positions[i + 1].valid = true;
        outgoingData.positions[i + 1].x = visionStates[i].xPos;
        outgoingData.positions[i + 1].y = visionStates[i].yPos;
        outgoingData.positions[i + 1].z = visionStates[i].zPos;
        outgoingData.positions[i + 1].timestamp = visionStates[i].timestamp;
    }
}

bool RobotOrbitTransmitter::sendRobotStates()
{
    PT_BEGIN();
    
    while (true)
    {
        PT_WAIT_UNTIL(messageTimer.execute() && odometry != nullptr);
        
        targetId = getAllyRobotId();  
        if (targetId == RefSerialData::RobotId::INVALID)
        {
            PT_YIELD();
            continue;
        }
        
        memcpy(&robotToRobotMessage.dataAndCRC16[0], &outgoingData, sizeof(PositionData));
        
        PT_CALL(refSerialTransmitter.sendRobotToRobotMsg(
            &robotToRobotMessage,
            ROBOT_ORBIT_MSG_ID,
            targetId,
            sizeof(PositionData)));
    }
    
    PT_END();
}

void RobotOrbitTransmitter::operator()(const DJISerial::ReceivedSerialMessage& message)
{
    memcpy(
        &incomingData,
        &message.data[sizeof(RefSerialData::Tx::InteractiveHeader)],
        sizeof(PositionData));
    
    RefSerialTransmitter::RobotId allyRobot = getAllyRobotId();
    if (allyRobot != RefSerialData::RobotId::INVALID && incomingData.positions[0].valid)
    {
        RobotState allyState;
        allyState.robotId = allyRobot;
        allyState.xPos = incomingData.positions[0].x;
        allyState.yPos = incomingData.positions[0].y;
        allyState.zPos = incomingData.positions[0].z;
        allyState.timestamp = incomingData.positions[0].timestamp;
        
        stateProvider.updateRobotState(allyRobot, allyState);
    }
    
    for (uint8_t i = 1; i <= MAX_TRACKED_ROBOTS; i++)
    {
        if (incomingData.positions[i].valid)
        {
            uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
            if (currentTime - incomingData.positions[i].timestamp > 5000)
            {
                continue;
            }
        }
    }
}

void RobotOrbitTransmitter::update()
{
    updateState();
    sendRobotStates();
}

}  // namespace aruwsrc::communication::serial
