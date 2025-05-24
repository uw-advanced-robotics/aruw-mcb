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
      visionCoprocessor(nullptr),
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
    
    if (visionCoprocessor != nullptr)
    {
        const auto& robotOrbits = visionCoprocessor->getLastRobotOrbitData();
        const auto& robotData = refSerial->getRobotData();
        bool isBlueTeam = RefSerial::isBlueTeam(robotData.robotId);
        uint32_t currentTime = tap::arch::clock::getTimeMilliseconds();
        
        for (int i = 0; i < aruwsrc::serial::VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
        {
            int robotType = robotOrbits.data[i].robotType;
            if (robotType == 0)
            {
                continue; 
            }
            
            RobotState robotState;
            robotState.robotId = getRobotIdFromType(robotType, isBlueTeam);
            robotState.xPos = robotOrbits.data[i].x;
            robotState.yPos = robotOrbits.data[i].y;
            robotState.zPos = robotOrbits.data[i].z;
            robotState.timestamp = currentTime;
            
            stateProvider.updateRobotState(robotState.robotId, robotState);
            
            uint8_t index = getRobotTypeIndex(robotType);
            if (index > 0 && index <= MAX_TRACKED_ROBOTS)
            {
                outgoingData.positions[index].valid = true;
                outgoingData.positions[index].x = robotState.xPos;
                outgoingData.positions[index].y = robotState.yPos;
                outgoingData.positions[index].z = robotState.zPos;
                outgoingData.positions[index].timestamp = robotState.timestamp;
            }
        }
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
            
            RefSerialData::RobotId robotId = RefSerialData::RobotId::INVALID;
            switch (i)
            {
                case 1:
                    robotId = RefSerial::isBlueTeam(allyRobot) ? 
                              RefSerialData::RobotId::RED_HERO : 
                              RefSerialData::RobotId::BLUE_HERO;
                    break;
                case 2:
                    robotId = RefSerial::isBlueTeam(allyRobot) ? 
                              RefSerialData::RobotId::RED_SOLDIER_3 : 
                              RefSerialData::RobotId::BLUE_SOLDIER_3;
                    break;
                case 3:
                    robotId = RefSerial::isBlueTeam(allyRobot) ? 
                              RefSerialData::RobotId::RED_SENTINEL : 
                              RefSerialData::RobotId::BLUE_SENTINEL;
                    break;
                default:
                    continue;
            }
            
            if (robotId != RefSerialData::RobotId::INVALID)
            {
                RobotState enemyState;
                enemyState.robotId = robotId;
                enemyState.xPos = incomingData.positions[i].x;
                enemyState.yPos = incomingData.positions[i].y;
                enemyState.zPos = incomingData.positions[i].z;
                enemyState.timestamp = incomingData.positions[i].timestamp;
                
                stateProvider.updateRobotState(robotId, enemyState);
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
