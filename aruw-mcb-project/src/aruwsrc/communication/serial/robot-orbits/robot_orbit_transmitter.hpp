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

#ifndef ROBOT_ORBIT_TRANSMITTER_HPP
#define ROBOT_ORBIT_TRANSMITTER_HPP

#include <modm/processing/resumable.hpp>
#include "robot_orbit_state.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/subsystem.hpp"
#include "aruwsrc/algorithms/odometry/chassis_kf_odometry.hpp"
#include "tap/architecture/clock.hpp"

using namespace aruwsrc::algorithms::odometry;
using namespace tap::communication::serial;

namespace aruwsrc::communication::serial {

class RobotOrbitTransmitter : public RefSerial::RobotToRobotMessageHandler {
public:
    RobotOrbitTransmitter(
        tap::Drivers* drivers, 
        RobotOrbitStateProvider& stateProvider, 
        ChassisKFOdometry* chassisOdometry, 
        RefSerial* refSerial
    );

    void sendRobotStates();
    void parseIncomingMessage(const DJISerial::ReceivedSerialMessage& message);
    void operator()(
        const DJISerial::ReceivedSerialMessage &message) override final;

private:
    tap::Drivers* drivers;
    RefSerialTransmitter serialTransmitter;
    RobotOrbitStateProvider& stateProvider;
    ChassisKFOdometry* odometry;
    RefSerial* refSerial;

    RefSerialTransmitter::RobotId getAllyRobotId() const;

    constexpr static uint8_t STATIC_CAST_SCALE_FACTOR = 100;

};

} // namespace aruwsrc::communication::serial

#endif // ROBOT_ORBIT_TRANSMITTER_HPP
