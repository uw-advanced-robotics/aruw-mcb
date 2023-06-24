/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef DRONE_TELEMETRY_HANDLER_HPP_
#define DRONE_TELEMETRY_HANDLER_HPP_

#include "tap/communication/serial/uart.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/communication/serial/mavlink_receiver.hpp"

namespace aruwsrc::drone
{
#define LOCAL_POSITION_NED_LEN 28
#define LOCAL_POSITION_NED_MSG_ID 32
#define MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC 185

#define COMMAND_INT_MSG_ID 32
#define COMMAND_SET_HOME_MSG_ID 32

struct LocalPositionNed
{
    uint32_t time_boot_ms;  // Timestamp (milliseconds since system boot)
    float x;                // X Position in NED frame in meters
    float y;                // Y Position in NED frame in meters
    float z;                // Z Position in NED frame in meters
    float vx;               // X Speed in NED frame in meter / s
    float vy;               // Y Speed in NED frame in meter / s
    float vz;               // Z Speed in NED frame in meter / s
} modm_packed;

// See https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_HOME for definition
struct DoSetHomeCommandDefault
{
    uint8_t target_system = 1;     // System ID
    uint8_t target_component = 1;  // Component ID
    uint8_t frame = 1;             // The coordinate system of the COMMAND, using 1 for LOCAL_NED
    uint16_t command = COMMAND_SET_HOME_MSG_ID;  // The scheduled action for the mission item.
    uint8_t current = 0;                         // unused, set to 0
    uint8_t autocontinue = 0;                    // unused, set to 0
    float useCurrent = 1;  // Use the current location, 1 is yes, 0 is the specified location
    float param2;          // Unused
    float param3;          // Unused
    float yaw = 0;         // Heading, NAN makes it used it's heading
    int32_t x = 0;         // local: x position
    int32_t y = 0;         // local: y position
    float z = 0;           // local: z position
} modm_packed;

/**
 * This class is used to get telemetry from the drone, following the Mavlink local_position_ned
 * message. This is built off the following message defintion:
 * https://github.com/mavlink/c_library_v2/blob/master/common/mavlink_msg_local_position_ned.h
 */
class DroneTelemetryHandler : public aruwsrc::communication::serial::MavlinkReceiver
{
public:
    DroneTelemetryHandler(tap::Drivers* drivers, tap::communication::serial::Uart::UartPort port);
    ~DroneTelemetryHandler() = default;

    void messageReceiveCallback(const ReceivedMavlinkMessage& completeMessage) override;
    LocalPositionNed* getLocalPositionNed();
    void setHomePosition();

#ifndef ENV_UNIT_TESTS
private:
#endif
    LocalPositionNed localPositionNed;
    MavlinkMessage<sizeof(DoSetHomeCommandDefault)> setHomeCommand;
    DoSetHomeCommandDefault cmd;
    uint16_t gotAMessage = 0;
    bool gotMsgAcknowledgment = false;
    bool wroteSetHomeCommand = false;
};

}  // namespace aruwsrc::drone

#endif
