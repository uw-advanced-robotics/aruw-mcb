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

#ifndef INTER_ROBOT_TRANSMITTER_HPP_
#define INTER_ROBOT_TRANSMITTER_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "modm/processing/protothread.hpp"

namespace aruwsrc::communication::inter_robot_comm
{
using namespace tap::communication::serial;
using namespace aruwsrc::serial;
class InterRobotTransmitter : public modm::pt::Protothread,
                              public RefSerial::RobotToRobotMessageHandler
{
public:
    InterRobotTransmitter(
        RefSerial* refSerial,
        RefSerialTransmitter* refSerialTransmitter,
        VisionCoprocessor* visionCoprocessor);

    /**
     * Sends the current orbit state to the ally robot.
     * Needs to be called repeatedly to ensure messages are sent.
     */
    bool sendMessage();

    /**
     * Updates state from vision coprocessor.
     * Needs to be called periodically to ensure the state is up-to-date.
     */
    void updateState();

    // Processes the received message from the ally robot.
    void operator()(const DJISerial::ReceivedSerialMessage& message) override;

    // Message structure for sending/receiving robot states
    struct EnemyRobotState
    {
        struct RobotState
        {
            bool current;  // Used to signal on the ally robot that this is the current state
            float x;
            float y;
            float z;
            uint16_t robotType; // icon if it has one
            uint32_t timestamp;  // Timestamp in milliseconds, used for processing "active" robots
        };
        RobotState robot[VisionCoprocessor::MAX_NUM_ROBOT_ORBITS];
    };

    const EnemyRobotState& getStateEstimate() const { return stateEstimate; }

private:
    // Needed state
    RefSerial* refSerial;
    RefSerialTransmitter* refSerialTransmitter;
    VisionCoprocessor* visionCoprocessor;
    RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;

    // Sending data
    uint16_t MSG_ID = 0x201;
    RefSerial::RobotId targetId;
    tap::arch::PeriodicMilliTimer timer{500};

    EnemyRobotState stateEstimate;

    // Debug stuff
    EnemyRobotState outgoingMessage;
    EnemyRobotState incomingMessage;
    int parsedMessageCount = 0;
    int ptLoopCount = 0;

    // Helper methods
    inline RefSerialTransmitter::RobotId getAllyRobotId() const;

    float POSITION_TOLERANCE = 0.75f;  // Meters
    /**
     * Assigns incoming robot state to the nearest robot state in the
     * current state estimate. If the incoming state is closer than
     * POSITION_TOLERANCE to the nearest robot state, than update that
     * state. Otherwise, update the state of the oldest robot state.
     */
    void updateNearestRobotState(EnemyRobotState::RobotState& state);
};
}  // namespace aruwsrc::communication::inter_robot_comm

#endif  // INTER_ROBOT_TRANSMITTER_HPP_
