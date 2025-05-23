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

#ifndef ROBOT_ORBIT_TRANSMITTER_HPP_
#define ROBOT_ORBIT_TRANSMITTER_HPP_

#include <modm/processing/resumable.hpp>

#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "aruwsrc/communication/serial/inter_robot_signal_transmitter.hpp"

#include "robot_orbit_state.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::communication::serial;

namespace aruwsrc::communication::serial
{
enum class RobotOrbitMessageType : uint8_t
{
    POSITION_UPDATE = 0,
    NUM_MESSAGE_TYPES
};

class RobotOrbitTransmitter : public RefSerial::RobotToRobotMessageHandler
{
public:
    RobotOrbitTransmitter(
        tap::Drivers* drivers,
        RobotOrbitStateProvider& stateProvider,
        RefSerial* refSerial);

    void operator()(const DJISerial::ReceivedSerialMessage& message) override final;
    inline void attachOdometry(Odometry2DInterface* odometry) { this->odometry = odometry; }
    void update();

private:
    tap::Drivers* drivers;
    RobotOrbitStateProvider& stateProvider;
    Odometry2DInterface* odometry = nullptr;
    RefSerial* refSerial;

    RefSerialTransmitter::RobotId getAllyRobotId() const;
    
    class OrbitMessageTransmitter : 
        public InterRobotSignalMessageTransmitter<
            RobotOrbitMessageType,
            static_cast<uint8_t>(RobotOrbitMessageType::NUM_MESSAGE_TYPES)>
    {
    public:
        OrbitMessageTransmitter(
            tap::Drivers& drivers,
            std::vector<tap::communication::serial::RefSerialData::RobotId>& targetIds,
            uint16_t messageId)
            : InterRobotSignalMessageTransmitter(drivers, targetIds, messageId) {}

        void setMessageData(RobotOrbitMessageType type, const uint8_t* data, size_t length);
    };

    // Define scale factor based on field size and uint16_t max value
    // RoboMaster field is 12m in the largest dimension
    // UINT16_MAX = 65535, dividing by 12 gives ~5461 units per meter
    static constexpr uint16_t STATIC_CAST_SCALE_FACTOR = UINT16_MAX / 12;

    std::vector<tap::communication::serial::RefSerialData::RobotId> targetRobots;
    OrbitMessageTransmitter messageTransmitter;

    void parseIncomingMessage(const DJISerial::ReceivedSerialMessage& message);
    void sendRobotStates();
};

}  // namespace aruwsrc::communication::serial

#endif  // ROBOT_ORBIT_TRANSMITTER_HPP_
