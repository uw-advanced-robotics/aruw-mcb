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
    enum RobotIndex : uint8_t
    {
        STANDARD = 0,
        HERO = 1,
        SENTRY = 2,
        NUM_ROBOTS = 3
    };

    struct EnemyRobotState
    {
        struct RobotState
        {
            bool current;  // Used to signal on the ally robot that this is the current state
            float x;
            float y;
            float z;
            uint32_t timestamp;  // Timestamp in milliseconds, used for processing "active" robots
        };
        RobotState robot[NUM_ROBOTS];
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
    inline RobotIndex getIndexFromRobotType(int robotType) const
    {
        switch (robotType)
        {
            case 1:
                return HERO;
            case 3:
                return STANDARD;
            case 4:
                return STANDARD;
            case 7:
                return SENTRY;
            default:
                return NUM_ROBOTS;  // Invalid type
        }
    }

    inline RefSerialTransmitter::RobotId getAllyRobotId() const
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
};

}  // namespace aruwsrc::communication::inter_robot_comm

#endif  // INTER_ROBOT_TRANSMITTER_HPP_
