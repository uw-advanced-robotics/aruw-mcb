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

#include "inter_robot_transmitter.hpp"

namespace aruwsrc::communication::inter_robot_comm
{
InterRobotTransmitter::InterRobotTransmitter(
    RefSerial* refSerial,
    RefSerialTransmitter* refSerialTransmitter,
    VisionCoprocessor* visionCoprocessor)
    : refSerial(refSerial),
      refSerialTransmitter(refSerialTransmitter),
      visionCoprocessor(visionCoprocessor)
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

        // Copy and reset
        memcpy(&robotToRobotMessage.dataAndCRC16[0], &outgoingMessage, sizeof(EnemyRobotState));
        memset(&outgoingMessage, 0, sizeof(EnemyRobotState));

        PT_CALL(refSerialTransmitter->sendRobotToRobotMsg(
            &robotToRobotMessage,
            MSG_ID,
            targetId,
            sizeof(EnemyRobotState)));

        ptLoopCount++;
    }
    PT_END();
}

void InterRobotTransmitter::operator()(const DJISerial::ReceivedSerialMessage& message)
{
    memcpy(
        &incomingMessage,
        &message.data[sizeof(RefSerialData::Tx::InteractiveHeader)],
        sizeof(EnemyRobotState));

    // Go through each robot state, if it is current, update the state estimate
    for (int i = 0; i < VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
    {
        if (incomingMessage.robot[i].current)
        {
            updateNearestRobotState(incomingMessage.robot[i]);
        }
    }

    parsedMessageCount++;
}

void InterRobotTransmitter::updateState()
{
    // Reset the outgoing message
    memset(&outgoingMessage, 0, sizeof(EnemyRobotState));

    // Go through each robot orbit we have and fill in the according state
    auto robotOrbits = visionCoprocessor->getLastRobotOrbitData();
    for (int i = 0; i < VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
    {
        // If all zeros, skip this orbit
        if (robotOrbits.data[i].x == 0.0f && robotOrbits.data[i].y == 0.0f &&
            robotOrbits.data[i].z == 0.0f)
        {
            continue;
        }

        EnemyRobotState::RobotState incomingRobotState = {
            .current = true,
            .x = robotOrbits.data[i].x,
            .y = robotOrbits.data[i].y,
            .z = robotOrbits.data[i].z,
            .timestamp = tap::arch::clock::getTimeMilliseconds()};

        // Update the outgoing message, broadcasts data as if the index is the robot type
        memcpy(&outgoingMessage.robot[i], &incomingRobotState, sizeof(EnemyRobotState::RobotState));

        // Update the current state estimate
        incomingRobotState.current = false;
        updateNearestRobotState(incomingRobotState);
    }
}

inline RefSerialTransmitter::RobotId InterRobotTransmitter::getAllyRobotId() const
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
                   ? RefSerialData::RobotId::BLUE_SOLDIER_1
                   : RefSerialData::RobotId::BLUE_HERO;
    }
    else
    {
        return (robotData.robotId == RefSerialData::RobotId::RED_HERO)
                   ? RefSerialData::RobotId::RED_SOLDIER_1
                   : RefSerialData::RobotId::RED_HERO;
    }
}

void InterRobotTransmitter::updateNearestRobotState(EnemyRobotState::RobotState& state)
{
    int nearestIndex = -1;
    int oldestIndex = 0;
    float nearestDistance = FLT_MAX;
    uint32_t oldestTimestamp = UINT32_MAX;
    for (int i = 0; i < VisionCoprocessor::MAX_NUM_ROBOT_ORBITS; i++)
    {
        const auto& currentState = stateEstimate.robot[i];
        float distance = sqrtf(
            (currentState.x - state.x) * (currentState.x - state.x) +
            (currentState.y - state.y) * (currentState.y - state.y) +
            (currentState.z - state.z) * (currentState.z - state.z));

        if (distance < nearestDistance && distance < POSITION_TOLERANCE)
        {
            nearestDistance = distance;
            nearestIndex = i;
        }

        if (currentState.timestamp < oldestTimestamp)
        {
            oldestTimestamp = currentState.timestamp;
            oldestIndex = i;
        }
    }

    if (nearestIndex != -1)
    {
        // Update the nearest state
        stateEstimate.robot[nearestIndex] = state;
        stateEstimate.robot[nearestIndex].timestamp = tap::arch::clock::getTimeMilliseconds();
    }
    else
    {
        // Update the oldest state
        stateEstimate.robot[oldestIndex] = state;
        stateEstimate.robot[oldestIndex].timestamp = tap::arch::clock::getTimeMilliseconds();
    }
}

}  // namespace aruwsrc::communication::inter_robot_comm
