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

#include <modm/processing/protothread.hpp>

#include "tap/algorithms/odometry/odometry_2d_interface.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "robot_orbit_state.hpp"

using namespace tap::algorithms::odometry;
using namespace tap::communication::serial;

namespace aruwsrc::communication::serial
{
class RobotOrbitTransmitter : public modm::pt::Protothread, 
                              public RefSerial::RobotToRobotMessageHandler
{
public:
    RobotOrbitTransmitter(
        tap::Drivers* drivers,
        RobotOrbitStateProvider& stateProvider,
        RefSerial* refSerial);

    bool sendRobotStates();

    void updateState();

    void operator()(const DJISerial::ReceivedSerialMessage& message) override;

    void update();

    inline void attachOdometry(Odometry2DInterface* odometry) { this->odometry = odometry; }

    inline void attachVisionCoprocessor(aruwsrc::serial::VisionCoprocessor* visionCoprocessor) 
    { 
        this->visionCoprocessor = visionCoprocessor; 
    }

private:
    struct PositionData
    {
        struct RobotPosition
        {
            bool valid;     
            float x;       
            float y;      
            float z;      
            uint32_t timestamp; 
        };

        RobotPosition positions[MAX_TRACKED_ROBOTS + 1];
    };

    tap::Drivers* drivers;
    RobotOrbitStateProvider& stateProvider;
    Odometry2DInterface* odometry = nullptr;
    aruwsrc::serial::VisionCoprocessor* visionCoprocessor = nullptr;
    RefSerial* refSerial;
    RefSerialTransmitter refSerialTransmitter;
    
    static constexpr uint16_t ROBOT_ORBIT_MSG_ID = 0x200;
    
    tap::arch::PeriodicMilliTimer messageTimer{500}; 

    RefSerialData::Tx::RobotToRobotMessage robotToRobotMessage;
    
    PositionData outgoingData;
    PositionData incomingData;
    
    RefSerialTransmitter::RobotId targetId;
    
    RefSerialTransmitter::RobotId getAllyRobotId() const;
    
    inline uint8_t getRobotTypeIndex(uint8_t robotType) const
    {
        switch (robotType)
        {
            case 1:
                return 1;
            case 3:
                return 2;
            case 4:
                return 2;
            case 7:
                return 3;
            default:
                return 0; 
        }
    }
    
    inline RefSerialData::RobotId getRobotIdFromType(uint8_t robotType, bool isBlueTeam) const
    {
        switch (robotType)
        {
            case 1:
                return isBlueTeam ? RefSerialData::RobotId::RED_HERO : RefSerialData::RobotId::BLUE_HERO;
            case 3: 
                return isBlueTeam ? RefSerialData::RobotId::RED_SOLDIER_3 : RefSerialData::RobotId::BLUE_SOLDIER_3;
            case 4: // standard 4 is actually standard 3
                return isBlueTeam ? RefSerialData::RobotId::RED_SOLDIER_3 : RefSerialData::RobotId::BLUE_SOLDIER_3;
            case 7: 
                return isBlueTeam ? RefSerialData::RobotId::RED_SENTINEL : RefSerialData::RobotId::BLUE_SENTINEL;
            default:
                return RefSerialData::RobotId::INVALID;
        }
    }
};

}  // namespace aruwsrc::communication::serial

#endif  // ROBOT_ORBIT_TRANSMITTER_HPP_
