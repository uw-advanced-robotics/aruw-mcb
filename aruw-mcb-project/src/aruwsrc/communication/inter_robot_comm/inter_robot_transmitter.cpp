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
    for (int i = 0; i < RobotIndex::NUM_ROBOTS; i++)
    {
        if (incomingMessage.robot[i].current)
        {
            stateEstimate.robot[i] = incomingMessage.robot[i];
            // Set timestamp to our current time
            stateEstimate.robot[i].timestamp = tap::arch::clock::getTimeMilliseconds();
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
        int robotType = robotOrbits.data[i].robotType;
        if (robotType == 0)
        {
            continue;  // Skip invalid robot types
        }
        RobotIndex index = getIndexFromRobotType(robotType);

        // Update the outgoing message
        EnemyRobotState::RobotState& outgoingRobotState =
            outgoingMessage.robot[static_cast<int>(index)];
        outgoingRobotState.current = true;
        outgoingRobotState.x = robotOrbits.data[i].x;
        outgoingRobotState.y = robotOrbits.data[i].y;
        outgoingRobotState.z = robotOrbits.data[i].z;
        outgoingRobotState.timestamp = tap::arch::clock::getTimeMilliseconds();

        // Update the current state estimate
        EnemyRobotState::RobotState& currentRobotState =
            stateEstimate.robot[static_cast<int>(index)];
        currentRobotState.x = robotOrbits.data[i].x;
        currentRobotState.y = robotOrbits.data[i].y;
        currentRobotState.z = robotOrbits.data[i].z;
        currentRobotState.timestamp = tap::arch::clock::getTimeMilliseconds();
    }
}
}  // namespace aruwsrc::communication::inter_robot_comm
